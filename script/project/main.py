from mobile_manipulator_unicycle import MobileManipulatorUnicycle
import time
from utils import action

robot = action()

robot.open_gripper()
robot.arm_forward()
robot.close_gripper()
robot.arm_backword()

time.sleep(1)

robot.arm_forward()
robot.open_gripper()
robot.arm_backword()
robot.close_gripper()

robot.set_arm_pose(0,0)



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