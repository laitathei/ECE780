from mobile_manipulator_unicycle import MobileManipulatorUnicycle
from utils import action
import numpy as np
from control import qp_solver, pickup_object, dropoff_object

robot = action()

# control parameter
alpha = 2 # higher when increase velocity (Proportional controller)
beta = 0.5 # lower when avoid the obstacle aggressively
k_f = 1 # higher when difficult to pointing to the target, lower when damping occur frequently
k_h = 0.2 # 0 means stop at the boundary, higher when not enough in avoid angle, lower when avoid angle too much
k_delta = 1 # gain of slack variable, higher when follow the constraints more strictly, lower when no need to follow the constraint too strictly
d = 0.5 # obstacle radius
L = 0.49 # distance between current position and the pickup position
L2 = 0.5 # distance between current position and the dropoff position
reach_pickup = False
reach_dropoff = False
task_done = False

while not task_done:
    # Get the robot's current pose.
    current_pos = []
    pickup = []
    dropoff = []

    # while current_pos == [] or pickup == [] or dropoff == []:
    poses = robot.get_pose()
    current_pos = poses[0]
    pickup = poses[1]
    dropoff = poses[2]

    current_pos = np.array(current_pos)
    pickup = np.array(pickup)
    dropoff = np.array(dropoff)
    obstacles = np.array([poses[4]])
    # pickup_object(robot)
    # exit()
    # dropoff_object(robot)
    # exit()
    
    Px, Py, theta = current_pos
    if reach_pickup == False and reach_dropoff == False:
        Px_d, Py_d, theta_d = pickup
    elif reach_pickup == True and reach_dropoff == False:
        Px_d, Py_d, theta_d = dropoff
    Px_o, Py_o, theta_o = obstacles[:, 0], obstacles[:, 1], obstacles[:, 2]

    s = np.array([[Px], [Py], [theta]]) # state vector [Px, Py, theta]
    sd = np.array([[Px_d], [Py_d], [theta_d]]) # target state [Px_d, Py_d, theta_d]
    so = np.array([Px_o, Py_o, theta_o]) # obstalce state [Px_o, Py_o, theta_o]
    print(np.linalg.norm(sd[:2]-s[:2]))
    u = qp_solver(s, sd, so, alpha, beta, d, k_f, k_h, k_delta)

    v = u[0]
    omega = u[1]

    if reach_pickup == False and reach_dropoff == False:
        if np.linalg.norm(sd[:2]-s[:2]) >= L:
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0, lapse=0.01)
        else:
            pickup_object(robot)
            reach_pickup = True
    elif reach_pickup == True and reach_dropoff == False:
        if np.linalg.norm(sd[:2]-s[:2]) >= L2:
            robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=0.0, lapse=0.01)
        else:
            dropoff_object(robot)
            reach_dropoff = True
    else:
        robot.set_mobile_base_speed_and_gripper_power(v=0, omega=0, gripper_power=0.0)
        task_done = True
        print("Finish task")
