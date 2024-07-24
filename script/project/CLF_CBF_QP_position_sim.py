from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from qpsolvers import solve_qp

# Two options to initialize the simulator
# (i) random robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1)
# (ii) specified robot and object locations
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[0, 0, 0.0], pickup_location=[0.75, 0.75, np.pi/2], dropoff_location=[-0.75, -0.75, 0], obstacles_location=[[0.5, 0.5, 0], [-0.5, -0.5, 0]])
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[2, 3, -np.pi/6], pickup_location=[-4, -2, 0], dropoff_location=[-0.75, -0.75, 0], obstacles_location=[[-1.0, -0.25, 0], [-0.5, -0.5, 0]])

# control parameter
alpha = 5 # higher when increase velocity (Proportional controller)
beta = 100 # higher when avoid the obstacle aggressively
k_f = 4 # higher when difficult to pointing to the target, lower when damping occur frequently
k_h = 0.5 # 0 means stop at the boundary, higher when not enough in avoid angle, lower when avoid angle too much
k_delta = 1 # gain of slack variable, higher when follow the constraints more strictly, lower when no need to follow the constraint too strictly
d = 0.2 # obstacle radius
L = 0.3 # distance between current position and the target position
reach_pickup = False
reach_dropoff = False
task_done = False
T = 60
start_time = time.time()

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
    h[0][0] = -alpha * (x_diff**2 + y_diff**2 + k_f * (Pd_theta_diff)**2)

    # obstacle avoid constraints
    for i in range (num_obstacles):
        P[2+i][2+i] = k_delta

        # xy position difference between current position and obstacle position
        xo_diff = so[0][i] - s[0][0]
        yo_diff = so[1][i] - s[1][0]
        xyo_diff = np.array([[xo_diff],
                             [yo_diff]])
        
        Po_theta = np.arctan2(yo_diff, xo_diff)
        Po_theta_diff = angle_diff(Po_theta+np.pi, theta)
        lo = ls(xo_diff, yo_diff)

        G[1+i][:2] = 2 * xyo_diff.T @ g[:2] + 2 * k_h * Po_theta_diff * lo.T @ g
        G[1+i][2+i] = 1
        G[0][2+i] = -1
        h[1+i][0] = beta * (xo_diff**2 + yo_diff**2 - d**2 - k_h * (Po_theta_diff)**2)

    x = solve_qp(P, q, G, h, solver="cvxopt")

    return x

while not task_done:
    # Get the robot's current pose.
    poses = robot.get_poses()
    print("")
    print("Current position: ", poses[0])
    print("Pickup position: ", poses[1])
    print("Dropoff position: ", poses[2])
    print("Obstacles position: ", poses[3])

    current_pos = poses[0]
    pickup = poses[1]
    dropoff = poses[2]
    obstacles = poses[3]

    current_pos = np.array(current_pos)
    pickup = np.array(pickup)
    dropoff = np.array(dropoff)
    obstacles = np.array(obstacles)

    Px, Py, theta = current_pos
    if reach_pickup == False and reach_dropoff == False:
        Px_d, Py_d, theta_d = pickup
    elif reach_pickup == True and reach_dropoff == False:
        Px_d, Py_d, theta_d = dropoff
    Px_o, Py_o, theta_o = obstacles[:, 0], obstacles[:, 1], obstacles[:, 2]

    s = np.array([[Px], [Py], [theta]])  # state vector [Px, Py, theta]
    sd = np.array([[Px_d], [Py_d], [theta_d]])  # target state [Px_d, Py_d, theta_d]
    so = np.array([Px_o, Py_o, theta_o]) # obstalce state [Px_o, Py_o, theta_o]

    u = qp_solver(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)

    v = u[0]
    omega = u[1]

    if reach_pickup == False and reach_dropoff == False:
        if np.linalg.norm(sd[:2]-s[:2]) >= L:
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0)
        else:
            reach_pickup = True
    elif reach_pickup == True and reach_dropoff == False:
        if np.linalg.norm(sd[:2]-s[:2]) >= L:
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0)
        else:
            reach_dropoff = True
    else:
        robot.set_mobile_base_speed_and_gripper_power(v=0, omega=0, gripper_power=0.0)
        task_done = True
        print("Finish task")


    