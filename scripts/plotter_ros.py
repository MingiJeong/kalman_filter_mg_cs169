#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt

COMMON_PATH = '/home/mingi/catkin_ws/src/kalman_filter_mg_cs169/csv/'
A = 'pose.csv'
B = 'cmd_estimate.csv'
C = 'pose_estimate.csv'
D = 'cmd_estimate_camera.csv'
E = 'pose_estimate_camera.csv'
F = 'ground_measure.csv'

PATH_POSE_A = COMMON_PATH + A
PATH_CMD_B = COMMON_PATH + B
PATH_POSE_ESTIMATE_C = COMMON_PATH + C
PATH_CMD_CAMERA_D = COMMON_PATH + D
PATH_POSE_CAMERA_E = COMMON_PATH + E
PATH_GND_F = COMMON_PATH + F


PATH_LIST = [PATH_POSE_A, PATH_CMD_B, PATH_POSE_ESTIMATE_C, PATH_CMD_CAMERA_D, PATH_POSE_CAMERA_E, PATH_GND_F]
COLOR = ['k', 'b', 'r', 'g', 'orange', 'm']
NAME = ['pose', 'cmd_vel estimate', 'pose estimate', 'cmd and camera', 'pose and camera', 'ground truth']
LINE_WIDTH = 3
#LINE_STYLE = ['-', '--', '-', '-', ':','-']
MARKER_STYLE = ['o', 's', 'o', 'x', 'D', '^']

def main():
    fig = plt.figure()
    for e, path in enumerate(PATH_LIST):
        x = []
        y = []

        with open(path,'r') as csvfile:
            plots = csv.reader(csvfile, delimiter=',')
            for row in plots:
                x.append(row[0])
                y.append(row[1])

        plt.plot(x,y, marker=MARKER_STYLE[e], color=COLOR[e], label= NAME[e], lw= LINE_WIDTH)

    plt.xlabel('Time [sec]', fontsize=30)
    plt.ylabel('Path [meter]', fontsize=30)
    plt.xlim(0,5)
    plt.xticks(fontsize=30)
    plt.yticks(fontsize=30)
    plt.title('Path comparison based on different estimate', fontsize=40)
    plt.legend(loc = 'lower right', fontsize=30)
    plt.show()

if __name__ =="__main__":
    rospy.init_node("plotting_node")
    main()
