import numpy as np

INITIAL_DIST_TO_WALL = 2

def kalman_calculator_cmd_vel(transition, dt, front_distance, x, P):
    F = np.eye(1) # relationship between state
    B = np.array([[dt]]) # time difference as input above
    U = np.array([[transition.linear.x]]) #veloicty

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

    #x = np.array([[1]]) # initial pose TODO: initial state input by user define
    #P = np.array([[2]])

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
