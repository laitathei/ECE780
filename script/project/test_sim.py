from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
import time
from qpsolvers import solve_qp
import numpy as np
# Two options to initialize the simulator
# (i) random robot and object locations
robot = MobileManipulatorUnicycleSim(robot_id=1)
# (ii) specified robot and object locations
# robot = MobileManipulatorUnicycleSim(robot_id=1, robot_pose=[1.0, 2.0, np.pi/4], pickup_location=[1.0, 1.0], dropoff_location=[-0.75, -0.75], obstacles_location=[[0.5, 0.5], [-0.5, -0.5], [0.5, -0.5]])

# # Move forward for 2 seconds.
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=0.5, omega=0.0, gripper_power=0.0)
#     time.sleep(0.05)

# # Move backward for 2 seconds.
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=-0.5, omega=0.0, gripper_power=0.0)
#     time.sleep(0.05)

# # Rotate CCW for 2 seconds.
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=10.0, gripper_power=0.0)
#     time.sleep(0.05)

# # Rotate CW for 2 seconds.
# start_time = time.time()
# while time.time() - start_time < 2.:
#     robot.set_mobile_base_speed_and_gripper_power(v=0.0, omega=-10.0, gripper_power=0.0)
#     time.sleep(0.05)

# # Stop the drive base and the gripper.
# robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)

# # Get the robot's current pose.
# poses = robot.get_poses()
# print(f"Robot, pickup, dropoff, obstacles poses: {poses}")


def clf_control(x, y, theta, x_d, y_d, k=1.0, k_theta=0.5):
    """
    CLF based controller for a mobile car.

    Parameters:
    x, y, theta : Current state of the car [position and orientation].
    x_d, y_d : Desired target position.
    k : Gain for velocity control.
    k_theta : Gain for angular velocity control.

    Returns:
    v, w : Control inputs [linear velocity, angular velocity].
    """
    # Error terms
    error_x = x_d - x
    error_y = y_d - y
    
    # Control law for linear velocity
    v = k * (error_x * np.cos(theta) + error_y * np.sin(theta))
    
    # Control law for angular velocity
    error_theta = np.arctan2(error_y, error_x) - theta
    # Normalize the angle to the range [-pi, pi]
    error_theta = (error_theta + np.pi) % (2 * np.pi) - np.pi
    
    w = k_theta * error_theta

    return v, w


def qp_clf_cbf_control_multi_obstacles(x, y, theta, x_d, y_d, obstacles, d_safe, k=1.0, k_theta=0.5, alpha=1.0, beta=1.0, gamma=1.0):
    P = np.eye(2)  # Minimize v^2 + w^2
    q = np.zeros(2)  # No linear term

    # Constraints setup for each obstacle
    num_obstacles = len(obstacles)
    G = []
    h_values = []
    
    # CLF constraint
    clf_vx = (x_d - x) * np.cos(theta) + (y_d - y) * np.sin(theta)
    G.append([-clf_vx, 0])  # -dotV
    clf_value = -gamma * ((x_d - x)**2 + (y_d - y)**2)  # CLF constraint
    h_values.append(clf_value)

    # CBF constraints for each obstacle
    for (x_o, y_o) in obstacles:
        cbf_vx = (x - x_o) * np.cos(theta) + (y - y_o) * np.sin(theta)
        G.append([2 * cbf_vx, 0])  # dotH
        cbf_value = -beta * ((x - x_o)**2 + (y - y_o)**2 - d_safe**2)  # CBF constraint
        h_values.append(cbf_value)

    G = np.array(G)
    h_values = np.array(h_values)
    
    try:
        # Solve QP problem
        [v, w] = solve_qp(P, q, G, h_values, solver='cvxopt')
    except ValueError as e:
        print(f"QP Solver Error: {e}")
        v, w = 0, 0  # Default safe stop in case of solver error
    except Exception as e:
        print(f"Unknown Error: {e}")
        v, w = 0, 0

    return v, w


x, y, theta = robot.get_poses()[0][0], robot.get_poses()[0][1], robot.get_poses()[0][2]
x_d, y_d = robot.pickup_location[0], robot.pickup_location[1]
obstacles = robot.obstacles_location

for _ in range(100000):
    v, w = qp_clf_cbf_control_multi_obstacles(x, y, theta, x_d, y_d, obstacles, d_safe=0.5)
    print(f"v: {v}, w: {w}")
    print(f"poses: {robot.get_poses()}")
    robot.set_mobile_base_speed_and_gripper_power(v, w, 0.0)
    poses = robot.get_poses()
    x, y, theta = poses[0][0], poses[0][1], poses[0][2]