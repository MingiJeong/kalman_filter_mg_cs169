#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/kalman_filter_mg_cs169/scripts/pose_estimate.py

import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from timeit import default_timer as timer
from kalman_calculator_cmd_vel import kalman_calculator_cmd_vel

INDEX = 0
MSG_INTERVAL_TIME = 0.5
# inital state (relative position) and error covariance for Kalman filter (based on robot foot frint)

class Kalman_filter_cmd_vel_laser():
    def __init__(self):
        self.cmd_subscriber = rospy.Subscriber("cmd_vel", Twist, self.cmd_callback)
        self.lidar_subscriber = rospy.Subscriber("scan", LaserScan, self.lidar_callback)
        self.state_publisher = rospy.Publisher("kalman_filter", PoseWithCovarianceStamped, queue_size=10)
        self.transition = None
        self.initial_pose = None
        self.initial_time_record_cmd = None
        self.time_record_cmd_now = None
        self.time_record_scan_now = None
        self.time_record_scan_list = []
        self.rate = rospy.Rate(15)
        self.initial_x = np.array([[0]]) # relative motion's inital position
        self.initial_P = None # bring up from inital_pose.py
        self.X_list = []
        self.P_list = []
        # self.ground truth

    # PoseWithCovarianceStamped: header input needed!
    # TODO: rosbag time syncronization
    # TODO: Kalman filter computation

    def cmd_callback(self, msg):
        if self.initial_time_record_cmd is not None:
            self.transition = msg
            self.time_record_cmd_now = rospy.get_time()
            #print("cmd_vel_input", transition, "time", self.time_record_cmd)

        # initial time for cmd_vel save
        else:
            self.transition = msg
            self.initial_time_record_cmd = rospy.get_time()
            self.time_record_cmd_now = self.initial_time_record_cmd
            #print("cmd_vel_input", transition, "time", self.time_record_cmd)

    def lidar_callback(self,msg):
        # under condition that cmd_vel is published after serial bridge is configured in order to calculate based on system model
        if self.initial_time_record_cmd is not None and rospy.get_time() >= self.initial_time_record_cmd:
            front_distance = msg.ranges[INDEX]
            self.time_record_scan_now = rospy.get_time()
            print("lidar_input", front_distance, type(front_distance), "time", self.time_record_scan_now)

            # dropping scan msg in case of inf (outlier)
            if len(self.time_record_scan_list) != 0 and front_distance != float("inf"):
                # time difference of scan messages (consecutive ones)
                self.time_record_scan_list.append(self.time_record_scan_now)
                time_difference_wrt_scan = self.time_record_scan_list[-1] - self.time_record_scan_list[-2]
                x, P = kalman_calculator_cmd_vel(self.transition, time_difference_wrt_scan, front_distance, self.X_list[-1], self.P_list[-1])
                self.X_list.append(x)
                self.P_list.append(P)
                #self.pose_with_covariance_publisher(x, P)

            # very first measurement condition and dropping scan msg in case of inf (outlier)
            elif len(self.time_record_scan_list) == 0 and front_distance != float("inf"):
                # initial time difference of scan message (based on the first cmd_vel)
                self.time_record_scan_list.append(self.time_record_scan_now)
                time_difference_wrt_scan = self.time_record_scan_now - self.initial_time_record_cmd
                print(self.initial_x, self.initial_P)
                x, P = kalman_calculator_cmd_vel(self.transition, time_difference_wrt_scan, front_distance, self.initial_x, self.initial_P)
                self.X_list.append(x)
                self.P_list.append(P)
                #self.pose_with_covariance_publisher(x, P)

            #print("time_difference_wrt_scan", time_difference_wrt_scan)

            #if self.time_record_scan_now >= self.time_record_cmd_now:
            #    time_difference_wrt_scan = self.time_record_scan_now - self.time_record_cmd_now

    def spin(self):

        # TODO: WRAP state publish function and when calculation finished. publish it
        state_message = PoseWithCovarianceStamped()
        # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html => row major list 6 x 6 matrix

        while not rospy.is_shutdown():
            # after receiving the first cmd_vel
            if len(self.X_list) != 0:
                #print("time_record", self.time_record_cmd)
                # state_message.header.seq =self.initial_pose.header.seq # header seq same as subscribed msg
                state_message.header.stamp = rospy.Time.now()
                state_message.header.frame_id = "odom_kf"

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
                print("publishing", state_message.header.stamp)

                if rospy.get_time() - self.time_record_cmd_now > MSG_INTERVAL_TIME:
                    scalar_X_list = []
                    scalar_P_list = []
                    for i in range(len(self.X_list)):
                        scalar_X_list.append(np.asscalar(self.X_list[i]))
                        scalar_P_list.append(np.asscalar(self.P_list[i]))
                    print("finished!")
                    print("X_list", scalar_X_list, "length", len(scalar_X_list))
                    print("P_list", scalar_P_list, "length", len(scalar_P_list))
                    # TODO plot graph or data accumulation
                    rospy.signal_shutdown("finish!")


    '''
    def pose_with_covariance_publisher(self, state, P):
        state_message = PoseWithCovarianceStamped()
        # http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseWithCovariance.html => row major list 6 x 6 matrix
        row_length = int(math.sqrt(len(state_message.pose.covariance)))

        for i in range(row_length):
            state_message.pose.covariance[i*row_length+i] = 1

        state_message.header.stamp = rospy.Time.now()
        state_message.header.frame_id = "odom_kf" # frame id same as subscribed msg

        state_message.pose.pose.position.x = state
        state_message.pose.pose.position.y = 0
        state_message.pose.pose.position.z = 0
        state_message.pose.pose.orientation.x =0
        state_message.pose.pose.orientation.y =0
        state_message.pose.pose.orientation.z =0
        state_message.pose.pose.orientation.w =1
        state_message.pose.covariance[0] = P

        self.state_publisher.publish(state_message)
        self.rate.sleep()
        print("publishing", state_message.header.stamp)

        if rospy.get_time() - self.time_record_cmd_now > 0.6:
            print("finish", self.X_list)
            # TODO plot graph or data accumulation
            rospy.signal_shutdown("finish!")




        def kalman_calculator(self, dt, front_distance, x, P):
            F = np.eye(1) # relationship between without control
            B = np.array([[dt]]) # time difference as input above
            U = np.array([[self.transition.linear.x]]) #veloicty

            H = np.array([[1]])

            Q = np.array([[0.1]])
            R = np.array([[1]])

            #x = np.array([[1]]) # initial pose TODO: initial state input by user define
            #P = np.array([[2]])

            # prediction
            xp = np.dot(F,x) + np.dot(B,U)
            Pp = np.dot(F, np.dot(P, np.transpose(F))) + Q

            # correction
            z_modified = np.dot(H, xp) # measure estimation
            r = (2-front_distance) - z_modified # actual measurement - measure estimation
            S = np.dot(H, np.dot(Pp, np.transpose(H))) + R
            K = np.dot(Pp, np.dot(np.transpose(H), np.linalg.inv(S)))
            x_new = xp + np.dot(K,r)
            P_new = Pp - np.dot(K, np.dot(H, Pp))

            print("kalman result", x_new, P_new)
            return (x_new, P_new)

    '''
    # Don't be confuse. initial_P comes from initial_pose.py
    # initial odom position is also saved in self.initial pose; however, initial_x is relative motion of foot frint
    def update_pose_msg(self):
        self.initial_pose = rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)
        self.initial_P = np.array([[self.initial_pose.pose.covariance[0]]])
        print("initial accepted", self.initial_P)

# while shutdown and spin difference question

def main():
    kalman_filter_cmd_vel_laser = Kalman_filter_cmd_vel_laser()
    kalman_filter_cmd_vel_laser.update_pose_msg()
    kalman_filter_cmd_vel_laser.spin()


if __name__ =="__main__":
    rospy.init_node("kalman_filter_cmd_vel_and_laser")
    main()
    #rospy.spin()
