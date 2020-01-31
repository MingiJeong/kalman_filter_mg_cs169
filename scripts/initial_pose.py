#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/kalman_filter_mg_cs169/scripts/initial_pose.py


import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from random import seed
from random import random

global launch_x, launch_P

class Initializer():
    def __init__(self):
        self.pose_publisher = rospy.Publisher("initialpose", PoseWithCovarianceStamped, queue_size=10)
        self.rate = rospy.Rate(5)

    def publish_function(self, initial_pose_msg, launch_x, launch_P):
        while not rospy.is_shutdown():
            initial_pose_msg.header.stamp = rospy.Time.now()
            initial_pose_msg.header.frame_id = "odom_kf"

            initial_pose_msg.pose.pose.position.x = launch_x
            initial_pose_msg.pose.pose.position.y = 0
            initial_pose_msg.pose.pose.position.z = 0
            initial_pose_msg.pose.pose.orientation.x =0
            initial_pose_msg.pose.pose.orientation.y =0
            initial_pose_msg.pose.pose.orientation.z =0
            initial_pose_msg.pose.pose.orientation.w =1
            initial_pose_msg.pose.covariance[0] = launch_P

            self.pose_publisher.publish(initial_pose_msg)
            self.rate.sleep()
            print("publishing initial pose", initial_pose_msg.pose.pose.position.x)


def main():
    initializer = Initializer()
    # seed random number generator
    #seed(1)
    # random pick generate random numbers [0,10)
    launch_x = np.array([[random() * 10]])
    # launch_P = np.array([[random() * 10]])
    launch_P = np.array([[3]])
    print("Initialized initial pose x:", launch_x, "initial pose covariance:", launch_P)

    initial_pose_msg = PoseWithCovarianceStamped()
    initializer.publish_function(initial_pose_msg, launch_x, launch_P)


if __name__ =="__main__":
    rospy.init_node("initial_pose_node")
    main()


# TODO TF is calculated by odom_kf - inital base_foot print
