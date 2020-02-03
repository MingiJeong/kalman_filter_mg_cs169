#!/usr/bin/env python3
import matplotlib.pyplot as plt
import csv


# ========== set up necessary parts ========== #
COMMON_PATH = '/home/mingi/catkin_ws/src/kalman_filter_mg_cs169/csv/'
PLOT_PATH = '/home/mingi/catkin_ws/src/kalman_filter_mg_cs169/plot/'
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
COLOR = ['k', 'b', 'r', 'orange','g', 'm']
NAME = ['pose', 'cmd_vel estimate', 'pose estimate', 'cmd and camera', 'pose and camera', 'ground truth']
LINE_WIDTH = 1
MARKER_SIZE = 3
#LINE_STYLE = ['-', '--', '-', '-', ':','-']
MARKER_STYLE = ['o', 's', 'o', 'D', 'x', '^']

A_VALUE = []
B_VALUE = []
C_VALUE = []
D_VALUE = []
E_VALUE = []
F_VALUE = []
VALUE_LIST = [A_VALUE, B_VALUE, C_VALUE, D_VALUE, E_VALUE, F_VALUE]

A_TIME = []
B_TIME = []
C_TIME = []
D_TIME = []
E_TIME = []
F_TIME = []
TIME_LIST = [A_TIME, B_TIME, C_TIME, D_TIME, E_TIME, F_TIME]

# ========== plot path comparison figure 1 ========== #
fig = plt.figure()
for e, path in enumerate(PATH_LIST):
    x = []
    y = []

    # plt by calling saved data from each path
    with open(path,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        for row in plots:
            x.append(row[0])
            y.append(row[1])

    # saving data for error comparison
    TIME_LIST[e].append(x[0])
    TIME_LIST[e].append(x[-1])
    VALUE_LIST[e].append(y[0])
    VALUE_LIST[e].append(y[-1])

    plt.plot(x,y, marker=MARKER_STYLE[e], markersize = MARKER_SIZE, color=COLOR[e], label= NAME[e], lw= LINE_WIDTH)

plt.xlabel('Time [sec]', fontsize=15)
plt.ylabel('Path [m]', fontsize=15)
plt.legend(loc = 'lower right', fontsize=10)
plt.xlim(-0.5,6.5)
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.title('Path comparison', fontsize=20)

fig.savefig(PLOT_PATH+'path_compare.pdf')

# ========== plot error comparison figure 2 ========== #

fig2 = plt.figure()
for k in range(len(VALUE_LIST)): # F_list is excluded as it is base.
    if k != 5:
        x_val = TIME_LIST[k]
        y_val = []
        error_start = float(VALUE_LIST[5][0]) - float(VALUE_LIST[k][0])
        y_val.append(error_start)
        error_end = float(VALUE_LIST[5][1]) - float(VALUE_LIST[k][1])
        y_val.append(error_end)

        plt.scatter(x_val, y_val, marker=MARKER_STYLE[k], color=COLOR[k], s = 20, label=NAME[k])
    else:
        x_val_base = TIME_LIST[5]
        y_val_base = VALUE_LIST[5]
        y_val_base[1] = 0
	
        plt.scatter(x_val_base, y_val_base, marker=MARKER_STYLE[5], color=COLOR[k], s = 20, label=NAME[5])
# bbox_inches ='tight'
plt.xlabel('Time [sec]', fontsize=15)
plt.ylabel('Error [m]', fontsize=15)
plt.legend(loc = 'lower left', fontsize=10)
plt.xlim(-0.5,6.5)
plt.ylim(-0.5, 0.5)
plt.xticks(fontsize=15)
plt.yticks(fontsize=15)
plt.title('Error comparison', fontsize=20)
fig2.savefig(PLOT_PATH+'error_compare.pdf')
plt.show()
