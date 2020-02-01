#!/usr/bin/env python

# make sure to execute the following lines at the terminal before running this py file
# source ~/catkin_ws/devel/setup.bash
# chmod +x catkin_ws/src/kalman_filter_mg_cs169/scripts/transformed_info_finder.py

import rospy
import math
from sensor_msgs.msg import LaserScan

class scanner:
    def __init__(self):
        self.receive = None

    def update_msg(self):
        self.receive = rospy.wait_for_message("transformed_depth_scan", LaserScan)
        incre = self.receive.angle_increment
        min = self.receive.angle_min
        max = self.receive.angle_max
        length = len(self.receive.ranges)
        angles = []
        answer = []
        for i in range(length):
            angle = math.degrees(min + incre * i)
            angles.append(angle)
            if abs(angle) < 0.03:
                answer.append(i)
        print("answer", answer)


def main():
    scan_finder = scanner()
    scan_finder.update_msg()


if __name__ =="__main__":
    rospy.init_node("transformed_info_finder")
    main()
