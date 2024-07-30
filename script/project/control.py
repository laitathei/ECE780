import numpy as np
from qpsolvers import solve_qp

def angle_diff(theta_d, theta):
    # Elimiate the effect of dtheta over +/- 180
    dtheta = theta_d - theta
    if dtheta >= np.pi:
        dtheta = -2*np.pi+dtheta
    elif dtheta <= -np.pi:
        dtheta = 2*np.pi+dtheta
    return dtheta

def gs(theta):
    s = np.array([[np.cos(theta), 0],
                  [np.sin(theta), 0],
                  [0, 1]])
    return s

def ls(x_diff, y_diff):
    d = x_diff**2 + y_diff**2 # Euclidean distance
    l = np.array([[y_diff/d],
                  [-x_diff/d],
                  [-1]])
    return l

def qp_solver(s, sd, so, alpha, beta, d, k_f, k_h, k_delta):
    """
    CLF and CBF combinated quadratic program
    """
    # heading theta
    theta = s[2][0]

    # g matrix
    g = gs(theta)

    # xy position difference between current position and target position
    x_diff = sd[0][0] - s[0][0] 
    y_diff = sd[1][0] - s[1][0]
    xy_diff = np.array([[x_diff],
                        [y_diff]])

    # angle difference between current positional theta and target position
    Pd_theta = np.arctan2(y_diff, x_diff)
    Pd_theta_diff = angle_diff(Pd_theta, theta)

    # l vector indicate relationship between current positoin and target position
    ld = ls(x_diff, y_diff)

    # minimize u and delta: [v omega delta] @ P @ [v omega delta].T + q.T @ [v omega delta].T = ||u||^2 + k_delta * delta^2
    # u = [v omega].T
    # x = [v omega delta].T
    # subject to Gx ≤ h, Ax = b, lb ≤ x ≤ ub

    num_obstacles = so.shape[1]
    P = np.zeros((2+num_obstacles,2+num_obstacles))
    q = np.zeros((2+num_obstacles,1))
    G = np.zeros((1+num_obstacles,2+num_obstacles))
    h = np.zeros((1+num_obstacles,1))

    # navigation constraints
    P[:2,:2] = np.eye(2)
    G[0][:2] = -2 * xy_diff.T @ g[:2] + 2 * k_f * Pd_theta_diff * ld.T @ g
    h[0][0] = -alpha * (x_diff**2 + y_diff**2 + k_f * Pd_theta_diff**2)

    # obstacle avoid constraints
    for i in range (num_obstacles):
        P[2+i][2+i] = k_delta

        # xy position difference between current position and obstacle position
        xo_diff = so[0][i] - s[0][0]
        yo_diff = so[1][i] - s[1][0]
        xyo_diff = np.array([[xo_diff],
                             [yo_diff]])
        
        Po_theta = np.arctan2(yo_diff, xo_diff)
        # Po_theta_diff = angle_diff(angle_diff(Po_theta, theta), -np.pi)
        Po_theta_diff = angle_diff(Po_theta-np.pi, theta)
        lo = ls(xo_diff, yo_diff)

        G[1+i][:2] = 2 * xyo_diff.T @ g[:2] + 2 * k_h * Po_theta_diff * lo.T @ g
        G[0][2+i] = -1
        h[1+i][0] = beta * (xo_diff**2 + yo_diff**2 - d**2 - k_h * Po_theta_diff**2)
    # print("")
    # print(P)
    # print(G)
    # print(h)
    x = solve_qp(P, q, G, h, solver="cvxopt")
    # print(x)
    return x

def pickup_object(robot):
    robot.open_gripper()
    robot.arm_forward()
    robot.close_gripper()
    robot.arm_backword()

def dropoff_object(robot):
    robot.arm_forward()
    robot.open_gripper()
    robot.arm_backword()
    robot.close_gripper()
    robot.set_arm_pose(0,0)