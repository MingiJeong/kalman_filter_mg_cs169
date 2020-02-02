#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/kalman_filter_mg_cs169/scripts/pose_estimate_camera.py

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from timeit import default_timer as timer
from kalman_calculator import *

INDEX = 324 # after analyzing from cmd_estimate_camera.launch for transformed_info_finder node
MSG_INTERVAL_TIME = 0.5
CSV_SAVE_PATH = '/home/mingi/catkin_ws/src/kalman_filter_mg_cs169/csv/pose_estimate_camera.csv'

class Kalman_filter_pose_camera():
    def __init__(self):
        self.cmd_subscriber = rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.pose_subscriber = rospy.Subscriber("pose", PoseStamped, self.pose_callback)
        self.camera_subscriber = rospy.Subscriber("transformed_depth_scan", LaserScan, self.camera_callback)
        self.state_publisher = rospy.Publisher("kalman_filter", PoseWithCovarianceStamped, queue_size=10)

        # initial values saving. The program is calculating based on the first time when cmd_vel is published.
        self.initial_pose = None
        self.initial_time_record_cmd = None # role as an initial time for analysis
        self.initial_time_record_pose = None

        # time saving variables
        self.time_record_cmd_now = None
        self.time_record_pose_now = None
        self.time_record_pose_list = []
        self.pose_msg_list = []
        self.pose_diff_list = []
        self.time_record_scan_now = None
        self.time_record_scan_list = []
        self.time_accumulation_scan_list = []
        self.first_calculation = False

        self.rate = rospy.Rate(15)

        # Even if the task is regarding pose,
        # inital state (relative position) and error covariance for Kalman filter (based on robot foot frint)
        self.initial_x = np.array([[0]]) # relative motion's inital position
        self.initial_P = None # bring up from inital_pose.py

        self.X_list = []
        self.P_list = []

    # Even if the task is regarding pose,
    # I keep subscribing to and using cmd_vel as the algorithm is based on the time when 1st cmd and last cmd was published
    def cmd_callback(self, msg):
        # from 2nd cmd_vel msg
        if self.initial_time_record_cmd is not None:
            self.time_record_cmd_now = rospy.get_time()

        # initial time for cmd_vel save
        else:
            self.initial_time_record_cmd = rospy.get_time()
            self.time_record_cmd_now = self.initial_time_record_cmd

    def pose_callback(self, msg):
        # pose calculation since initial cmd_vel was published
        if self.initial_time_record_cmd is not None and rospy.get_time() >= self.initial_time_record_cmd:
            if self.initial_time_record_pose is not None:
                self.pose_msg_list.append(msg)
                self.time_record_pose_now = rospy.get_time()
                self.time_record_pose_list.append(self.time_record_pose_now)
                current_msg = self.pose_msg_list[-1]
                previous_msg = self.pose_msg_list[-2]
                time_difference = self.time_record_pose_list[-1] - self.time_record_pose_list[-2]
                dist_for_Xp_calc = math.sqrt((current_msg.pose.position.x - previous_msg.pose.position.x)**2 + (current_msg.pose.position.y - previous_msg.pose.position.y)**2)
                self.pose_diff_list.append((time_difference, dist_for_Xp_calc))

            # first pose msg receives after cmd_vel published
            else:
                self.pose_msg_list.append(msg)
                self.initial_time_record_pose = rospy.get_time()
                self.time_record_pose_now = self.initial_time_record_pose
                self.time_record_pose_list.append(self.time_record_pose_now)
                time_difference = self.time_record_pose_now - self.initial_time_record_cmd
                self.pose_diff_list.append((time_difference, 0)) # pose still 0


    def camera_callback(self, msg):
        # under condition that cmd_vel is published after serial bridge is configured in order to calculate based on system model(pose)
        if self.initial_time_record_cmd is not None and rospy.get_time() >= self.initial_time_record_cmd:
            camera_distance = []
            for i in range(INDEX-20, INDEX+20):
                if not math.isnan(msg.ranges[i]):
                    camera_distance.append((msg.ranges[i]))

            # filtering out (outlier when camera_distance NaN)
            if len(camera_distance) != 0:
                front_distance = float(sum(camera_distance) / len(camera_distance))
                self.time_record_scan_now = rospy.get_time()

                # after 1st pose msg recieved and dropping scan msg in case of inf (outlier)
                if len(self.time_record_pose_list) != 0 and front_distance != float("inf"):
                    # time difference of scan messages (consecutive ones)
                    self.time_record_scan_list.append(self.time_record_scan_now)
                    time_difference_wrt_scan = self.time_record_scan_now - self.time_record_pose_list[-1]
                    base_time_difference = self.pose_diff_list[-1][0]
                    base_distance = self.pose_diff_list[-1][1]

                    # interpolation
                    transition = time_difference_wrt_scan * (base_distance/base_time_difference)

                    # from second calculation
                    if self.first_calculation == True:
                        x, P = kalman_calculator_pose_camera(transition, front_distance, self.X_list[-1], self.P_list[-1])
                        self.X_list.append(x)
                        self.P_list.append(P)
                        self.time_accumulation_scan_list.append(self.time_accumulation_scan_list[-1] + (self.time_record_scan_list[-1] - self.time_record_scan_list[-2]))

                    # very first calculation
                    else:
                        x, P = kalman_calculator_pose_camera(transition, front_distance, self.initial_x, self.initial_P)
                        self.X_list.append(x)
                        self.P_list.append(P)
                        self.first_calculation = True
                        self.time_accumulation_scan_list.append(self.time_record_scan_now - self.initial_time_record_cmd)

                elif len(self.time_record_pose_list) == 0: # when length 0
                    print("pose not yet received!")


    def spin(self):
        state_message = PoseWithCovarianceStamped()
        # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html => row major list 6 x 6 matrix

        while not rospy.is_shutdown():
            # after receiving the first cmd_vel
            if len(self.X_list) != 0:
                state_message.header.stamp = rospy.Time.now()
                state_message.header.frame_id = "odom_kf"
                # Based on Task 1's graduate student section, after we receive initialpose randomly,
                # It is reflected here by making PoseWithCovarianceStamped msg published through the topic "kalman_filter"
                # This msg is transformed as to base_footprint. You can check it on Rviz
                state_message.pose.pose.position.x = self.X_list[-1] + self.initial_pose.pose.pose.position.x
                state_message.pose.pose.position.y = 0
                state_message.pose.pose.position.z = 0
                state_message.pose.pose.orientation.x =0
                state_message.pose.pose.orientation.y =0
                state_message.pose.pose.orientation.z =0
                state_message.pose.pose.orientation.w =1
                state_message.pose.covariance[0] = self.P_list[-1]

                self.state_publisher.publish(state_message)
                self.rate.sleep()

                if rospy.get_time() - self.time_record_cmd_now > MSG_INTERVAL_TIME:
                    scalar_X_list = []
                    scalar_P_list = []
                    # Task 2 - E: path based on pose and camera depth image
                    for i in range(len(self.X_list)):
                        scalar_X_list.append(np.asscalar(self.X_list[i]))
                        scalar_P_list.append(np.asscalar(self.P_list[i]))

                    # Data check on screen
                    print("finished!")
                    print("X_list", scalar_X_list, "length", len(scalar_X_list))
                    print("X_time", self.time_accumulation_scan_list, "length", len(self.time_accumulation_scan_list))
                    # print("P_list", scalar_P_list, "length", len(scalar_P_list))

                    # file save function
                    csv_data_saver(CSV_SAVE_PATH, self.time_accumulation_scan_list, scalar_X_list)

                    rospy.signal_shutdown("finish!")


    # Don't be confused. initial_P comes from initial_pose.py
    # initial odom position is also saved in self.initial pose; however, initial_x is relative motion of foot frint
    def update_pose_msg(self):
        self.initial_pose = rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)
        self.initial_P = np.array([[self.initial_pose.pose.covariance[0]]])
        print("initial accepted", self.initial_P)


def main():
    kalman_filter_pose_camera = Kalman_filter_pose_camera()
    kalman_filter_pose_camera.update_pose_msg()
    kalman_filter_pose_camera.spin()


if __name__ =="__main__":
    rospy.init_node("kalman_filter_pose_and_camera")
    main()
