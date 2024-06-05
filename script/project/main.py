from mobile_manipulator_unicycle import MobileManipulatorUnicycle
import time
from utils import action

user_action = action()
user_action.manual_control()
# user_action.set_arm_pose(100,0)
# user_action.set_arm_pose(0,0)
# user_action.set_arm_pose(0,100)
# user_action.set_arm_pose(0,0)
# user_action.get_pose()

# while not task_done:

    # Get the robot's current pose.
    # poses = robot.get_poses()
    # print(f"Robot, pickup, dropoff, obstacles poses: {poses}")
    # compute control inputs (v, omega, gripper power, arm pose, leds)
    # ...
    
    # send control inputs to the robot
    # robot.set_mobile_base_speed_and_gripper_power(v=0.5, omega=0.0, gripper_power=1.0)
    # robot.set_arm_pose(100, 0)
    # robot.set_leds(255, 128, 64)