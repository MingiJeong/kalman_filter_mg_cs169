import numpy as np
import csv

# starting position's distance to the wall as per PA -1 request
INITIAL_DIST_TO_WALL = 2

# csv data saving function after all filtering is finished
def csv_data_saver(PATH_N_FILENAME, timelist, datalist):
    with open(PATH_N_FILENAME, 'w') as file:
        writer = csv.writer(file)
        writer.writerows(zip(timelist, datalist))
        print("csv file saved!")


def kalman_calculator_cmd_vel(transition, dt, front_distance, x, P):
    F = np.eye(1) # relationship between state
    B = np.array([[dt]]) # time difference as input above
    U = np.array([[transition.linear.x]]) #veloicty

    H = np.array([[1]])

    Q = np.array([[0.1]])
    R = np.array([[1]])

    # prediction
    xp = np.dot(F,x) + np.dot(B,U)
    Pp = np.dot(F, np.dot(P, np.transpose(F))) + Q

    # correction
    z_modified = np.dot(H, xp) # measure estimation
    r = (INITIAL_DIST_TO_WALL-front_distance) - z_modified # actual measurement - measure estimation
    S = np.dot(H, np.dot(Pp, np.transpose(H))) + R
    K = np.dot(Pp, np.dot(np.transpose(H), np.linalg.inv(S)))
    x_new = xp + np.dot(K,r)
    P_new = Pp - np.dot(K, np.dot(H, Pp))

    print("kalman result", x_new, P_new)
    return (x_new, P_new)

def kalman_calculator_cmd_vel_camera(transition, dt, front_distance, x, P):
    F = np.eye(1) # relationship between state
    B = np.array([[dt]]) # time difference as input above
    U = np.array([[transition.linear.x]]) #veloicty

    H = np.array([[1]])

    Q = np.array([[3]])
    R = np.array([[3]])


    # prediction
    xp = np.dot(F,x) + np.dot(B,U)
    Pp = np.dot(F, np.dot(P, np.transpose(F))) + Q

    # correction
    z_modified = np.dot(H, xp) # measure estimation
    r = (INITIAL_DIST_TO_WALL-front_distance) - z_modified # actual measurement - measure estimation
    S = np.dot(H, np.dot(Pp, np.transpose(H))) + R
    K = np.dot(Pp, np.dot(np.transpose(H), np.linalg.inv(S)))
    x_new = xp + np.dot(K,r)
    P_new = Pp - np.dot(K, np.dot(H, Pp))

    print("kalman result", x_new, P_new)
    return (x_new, P_new)

def kalman_calculator_pose(transition, front_distance, x, P):
    F = np.eye(1) # relationship between state
    B = np.array([[1]]) # already transition contains time factor; thus no need to make a control model such as t*v
    U = np.array([[transition]]) # delta x prediction (due to interpolation, I also reflected time factor before input here)

    H = np.array([[1]])

    Q = np.array([[0.1]])
    R = np.array([[1]])


    # prediction
    xp = np.dot(F,x) + np.dot(B,U)
    Pp = np.dot(F, np.dot(P, np.transpose(F))) + Q

    # correction
    z_modified = np.dot(H, xp) # measure estimation
    r = (INITIAL_DIST_TO_WALL-front_distance) - z_modified # actual measurement - measure estimation
    S = np.dot(H, np.dot(Pp, np.transpose(H))) + R
    K = np.dot(Pp, np.dot(np.transpose(H), np.linalg.inv(S)))
    x_new = xp + np.dot(K,r)
    P_new = Pp - np.dot(K, np.dot(H, Pp))

    print("kalman result", x_new, P_new)
    return (x_new, P_new)


def kalman_calculator_pose_camera(transition, front_distance, x, P):
    F = np.eye(1) # relationship between state
    B = np.array([[1]]) # already transition contains time factor; thus no need to make a control model such as t*v
    U = np.array([[transition]]) # delta x prediction (due to interpolation, I also reflected time factor before input here)

    H = np.array([[1]])

    Q = np.array([[3]])
    R = np.array([[3]])


    # prediction
    xp = np.dot(F,x) + np.dot(B,U)
    Pp = np.dot(F, np.dot(P, np.transpose(F))) + Q

    # correction
    z_modified = np.dot(H, xp) # measure estimation
    r = (INITIAL_DIST_TO_WALL-front_distance) - z_modified # actual measurement - measure estimation
    S = np.dot(H, np.dot(Pp, np.transpose(H))) + R
    K = np.dot(Pp, np.dot(np.transpose(H), np.linalg.inv(S)))
    x_new = xp + np.dot(K,r)
    P_new = Pp - np.dot(K, np.dot(H, Pp))

    print("kalman result", x_new, P_new)
    return (x_new, P_new)
