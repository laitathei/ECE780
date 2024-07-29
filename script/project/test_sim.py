from mobile_manipulator_unicycle_sim import MobileManipulatorUnicycleSim
import time
from qpsolvers import solve_qp
import numpy as np
# Two options to initialize the simulator
# (i) random robot and object locations
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

def get_g(theta):
    return np.c_[[np.cos(theta), np.sin(theta), 0], [0, 0, 1]]

def angle_diff(th1, th2):
    return np.arctan2(np.sin(th1 - th2), np.cos(th1 - th2))


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

KAPPA = 2.5
KAPPA_H = 0.5
KAPPA_DELTA = 0.5
ALPHA = 1.0
BETA = 1.0
def clf_cbf_control( robot_pose, destination_location, obstacles_location, d_safe=1.0):
    
    g = get_g(robot_pose[2])
    q = robot_pose
    q_des = np.array(destination_location)
    q_undes = np.array(obstacles_location)
    q_diff = q_des[:2] - q[:2]
    x_diff = q_diff[0]
    y_diff = q_diff[1]
    q_u_diff = q_undes - np.array([q[:2]] * 3)  
    x_u_diff = q_u_diff[:,0]
    y_u_diff = q_u_diff[:,1]

    one_over_x_squared_plus_y_squared = 1 / (x_diff ** 2 + y_diff ** 2)
    one_over_x_squared_plus_y_squared_u = 1 / (x_u_diff ** 2 + y_u_diff ** 2)

    heading_diff = angle_diff(np.arctan2(y_diff, x_diff), q[2])
    heading_u_diff = angle_diff(np.arctan2(y_u_diff, x_u_diff), q[2]+np.pi)

    Q = np.zeros((3, 3))
    Q[:2,:2] = np.eye(2)
    Q[2,2] = KAPPA_DELTA

    G = np.zeros((4, 3))
    G[0,:2] = -2 * q_diff.T @ g[:-1,:] + 2 * KAPPA * heading_diff * np.array([y_diff * one_over_x_squared_plus_y_squared, -x_diff * one_over_x_squared_plus_y_squared, -1]) @ g
    G[1:,:2] = 2 * q_u_diff @ g[:-1,:] + 2 * KAPPA_H * heading_u_diff * np.array([y_u_diff * one_over_x_squared_plus_y_squared_u, -x_u_diff * one_over_x_squared_plus_y_squared_u, [-1]*3]) @ g
    G[0, 2] = 1.0
    G[1:,2] = 0.0

    h = np.zeros(4)
    h[0] = -ALPHA * (np.linalg.norm(q_diff)**2 + KAPPA * heading_diff**2)
    h[1:4] = BETA * (np.linalg.norm(q_u_diff)**2 - d_safe**2 - KAPPA_H * heading_u_diff**2)
    u_delta = solve_qp(Q,
                    np.zeros((3,1)),
                    G[:,:],
                    h[:],
                    solver="cvxopt")
    return u_delta[:2] if u_delta is not None else (0, 0)

def clf_cbf_control_single_obstacle(robot_pose, destination_location, obstacles_location, d_safe=0.5):

    g = get_g(robot_pose[2])
    q = np.array(robot_pose)
    D = d_safe
    q_undes = np.array(obstacles_location)
    q_des = np.array(destination_location)
    q_diff = np.array([q_des[0] - q[0], q_des[1] - q[1]])
    x_diff = q_diff[0]
    y_diff = q_diff[1]
    q_u_diff = np.array([q_undes[0] - q[0], q_undes[1] - q[1]])
    x_u_diff = q_u_diff[0]
    y_u_diff = q_u_diff[1]
    one_over_x_squared_plus_y_squared = 1 / (x_diff ** 2 + y_diff ** 2)
    one_over_x_squared_plus_y_squared_u = 1 / (x_u_diff ** 2 + y_u_diff ** 2)
    heading_diff = angle_diff(np.arctan2(y_diff, x_diff), q[2])
    heading_u_diff = angle_diff(np.arctan2(y_u_diff, x_u_diff)+np.pi, q[2])
    Q = np.zeros((3, 3))
    Q[:2,:2] = np.eye(2)
    Q[2,2] = KAPPA_DELTA
    G = np.zeros((2, 3))
    G[0,:2] = -2 * q_diff.T @ g[:-1,:] + 2 * KAPPA * heading_diff * np.array([y_diff * one_over_x_squared_plus_y_squared, -x_diff * one_over_x_squared_plus_y_squared, -1]) @ g
    G[1,:2] = 2 * q_u_diff.T @ g[:-1,:] + 2 * KAPPA_H * heading_u_diff * np.array([y_u_diff * one_over_x_squared_plus_y_squared_u, -x_u_diff * one_over_x_squared_plus_y_squared_u, -1]) @ g
    G[0,2] = 1.0
    G[1,2] = 0.0
    h = np.zeros((2, 1))
    h[0] = -2 * (np.linalg.norm(q_diff)**2 + KAPPA * heading_diff**2)
    h[1] = 50 * (np.linalg.norm(q_u_diff)**2 - D**2 - KAPPA_H * heading_u_diff**2)
    u_delta = solve_qp(Q,
                 np.zeros((3,1)),
                 G[:,:],
                 h[:],
                 solver="cvxopt")
    return u_delta[:2] if u_delta is not None else (0, 0)

    


if __name__ == "__main__":

    for _ in range(10):
        robot = MobileManipulatorUnicycleSim(
            robot_id=1, 
            robot_pose=[2.0,1.0, np.random.uniform(-np.pi, np.pi)], 
            pickup_location=[-4.0, -2.0],
            dropoff_location=[-0.75, -0.75], 
            obstacles_location=[[-1.0, -0.25], [-0.5, -0.5], [0.5, -0.5]]
        )
        start_time = time.time()    
        while time.time() - start_time < 5.:
            pose = np.array(robot.get_poses()[0])
            pickup = np.array(robot.pickup_location)
            dropoff = np.array(robot.dropoff_location)
            obstacles = np.array(robot.obstacles_location)

            v, w = clf_cbf_control(pose, pickup , obstacles)
            print(v, w)
            if v and w:
                robot.set_mobile_base_speed_and_gripper_power(v, w, 0.0)
            else:
                robot.set_mobile_base_speed_and_gripper_power(0.1, 0.0, 0.0)
            
            if np.linalg.norm(pose[:2] - pickup[:2]) < 0.1:
                print("Picked up the object")
                break

            # while True:


            #     if np.linalg.norm(pose - pickup) < 0.1:
            #         print("Picked up the object")
            #         break
            
            # while True:
            #     v, w = clf_cbf_control(pose, dropoff , obstacles)
            #     robot.set_mobile_base_speed_and_gripper_power(v, w, 0.0)
            #     if np.linalg.norm(pose - dropoff) < 0.1:
            #         print("Dropped off the object")
            #         break

    quit()