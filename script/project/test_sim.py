from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
import time
import math
import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
from qpsolvers import solve_qp

# Two options to initialize the simulator
# (i) random robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1)
# (ii) specified robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[0, 0, 0.0], pickup_location=[0.75, 0.75, np.pi/2], dropoff_location=[-0.75, -0.75, 0], obstacles_location=[[0.5, 0.5, 0], [-0.5, -0.5, 0]])

start_time = time.time()
alpha = 0.5
T = 60

v_record = []
omega_record = []
time_record = []

def g(theta):
    return np.array([[np.cos(theta),0], 
                     [np.sin(theta),0], 
                     [0,1]])

def angle_diff(theta, theta_d):
    # Elimiate the effect of dtheta over +/- 180
    dtheta = theta - theta_d
    return np.arctan2(np.sin(dtheta), np.cos(dtheta))

while time.time() - start_time < T:
    # Get the robot's current pose.
    poses = robot.get_poses()
    print("")
    print("Current position: ", poses[0])
    print("Pickup position: ", poses[1])
    print("Heading: ", np.degrees(poses[0][2]))
    # print("Dropoff position: ", poses[2])
    # print("Obstacles position: ", poses[3])

    current_pos = poses[0]
    pickup = poses[1]
    dropoff = poses[2]
    obstacles = poses[3]

    current_pos = np.array(current_pos)
    pickup = np.array(pickup)
    Px, Py, theta = current_pos
    Px_d, Py_d, theta_d = pickup

    x_diff = Px - Px_d
    y_diff = Py - Py_d
    theta_diff = angle_diff(np.arctan2(y_diff, x_diff), theta)
    ds = np.array([[x_diff],
                   [y_diff],
                   [theta_diff]])

    u_star = -alpha/2 * np.linalg.pinv(g(theta)) @ (ds)

    v = u_star[0][0]
    omega = u_star[1][0]
    print(u_star)
    robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0)
    time.sleep(0.1)