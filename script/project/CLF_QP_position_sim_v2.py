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

# control parameter
alpha = 10
k_f = 0.5
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

def qp_solver(s, sd, alpha, k_f):
    theta = s[2][0]
    g = gs(theta)
    x_diff = sd[0][0] - s[0][0]
    y_diff = sd[1][0] - s[1][0]
    xy_diff = np.array([[x_diff],
                        [y_diff]])
    Pd_theta = np.arctan2(y_diff, x_diff)
    Pd_theta_diff = angle_diff(Pd_theta, theta)
    ld = ls(x_diff, y_diff)

    # minimize u: u.T @ P @ u + q.T @ u = ||u||^2 
    # subject to Gu ≤ h, Au = b, lb ≤ u ≤ ub
    P = np.eye(2)
    q = np.zeros((2,1))
    G = -2 * xy_diff.T @ g[:2] + 2 * k_f * Pd_theta_diff * ld.T @ g
    h = np.array([-alpha * (x_diff**2 + y_diff**2 + k_f * (Pd_theta_diff)**2)])
    u = solve_qp(P, q, G, h, solver="cvxopt")
    return u

while time.time() - start_time < T:
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
    Px, Py, theta = current_pos
    Px_d, Py_d, theta_d = pickup

    s = np.array([[Px], [Py], [theta]])  # state vector [Px, Py, theta]
    sd = np.array([[Px_d], [Py_d], [theta_d]])  # target state [Px_d, Py_d, theta_d]
    u = qp_solver(s, sd, alpha, k_f)
    v = u[0]
    omega = u[1]

    robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0)