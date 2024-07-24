from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
import numpy as np
import matplotlib.pyplot as plt
from control import qp_solver

# Two options to initialize the simulator
# (i) random robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1)
# (ii) specified robot and object locations
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[0, 0, 0.0], pickup_location=[0.75, 0.75, np.pi/2], dropoff_location=[-0.75, -0.75, 0], obstacles_location=[[0.5, 0.5, 0], [-0.5, -0.5, 0]])
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[2, 3, -np.pi/6], pickup_location=[-4, -2, 0], dropoff_location=[-0.75, -0.75, 0], obstacles_location=[[-1.0, -0.25, 0], [-0.5, -0.5, 0]])

# control parameter
alpha = 5 # higher when increase velocity (Proportional controller)
beta = 1 # higher when avoid the obstacle aggressively
k_f = 10 # higher when difficult to pointing to the target, lower when damping occur frequently
k_h = 0 # 0 means stop at the boundary, higher when not enough in avoid angle, lower when avoid angle too much
k_delta = 0.3 # gain of slack variable, higher when follow the constraints more strictly, lower when no need to follow the constraint too strictly
d = 0.2 # obstacle radius
L = 0.4 # distance between current position and the target position
reach_pickup = False
reach_dropoff = False
task_done = False

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

    s = np.array([[Px], [Py], [theta]]) # state vector [Px, Py, theta]
    sd = np.array([[Px_d], [Py_d], [theta_d]]) # target state [Px_d, Py_d, theta_d]
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


    