from mobile_manipulator_unicycle import MobileManipulatorUnicycle
from pynput.keyboard import Key, Listener
import time

class action:
    def __init__(self) -> None:
        self.robot = MobileManipulatorUnicycle(robot_id=2, backend_server_ip="192.168.0.2")
        time.sleep(1)
        self.v = 0.5
        self.omega = 10.0
        self.gripper_power = 1.0
        self.x = 0.0
        self.y = 0.0
        self.arm_position_step = 10.0
        self.power_step = 1.0
        print("Initialize action finish")

    def set_mobile_base_speed_and_gripper_power(self,  v, omega, gripper_power, lapse = 3.):
        start_time = time.time()
        while time.time() - start_time < lapse:
            self.robot.set_mobile_base_speed_and_gripper_power(v=v, omega=omega, gripper_power=gripper_power)
            time.sleep(0.05)
        print("set_mobile_base_speed_and_gripper_power finish")

    def set_arm_pose(self, x, y,lapse = 3.):
        """
        Request robot arm move to desired position

        :param float x: desired position in x-axis
        :param float y: desired position in y-axis
        """
        start_time = time.time()
        while time.time() - start_time < lapse:
            self.robot.set_arm_pose(x, y)
            time.sleep(0.05)
        print("set_arm_pose finish")

    def move_forward(self, v, time):
        """
        Request robot to move forward with specific time

        :param float v: desired robot velocity in m/s
        :param float time: time for robot move forward
        """
        start_time = time.time()
        while time.time() - start_time < time:
            self.robot.set_mobile_base_speed_and_gripper_power(v=v, omega=0, gripper_power=0)
            time.sleep(0.05)
        print("Move forward action finish")

    def move_backward(self, v, time):
        """
        Request robot to move backward with specific time

        :param float v: desired robot velocity in m/s
        :param float time: time for robot move backward
        """
        start_time = time.time()
        while time.time() - start_time < time:
            self.robot.set_mobile_base_speed_and_gripper_power(v=-v, omega=0, gripper_power=0)
            time.sleep(0.05)
        print("Move backward action finish")

    def rotate_clockwise(self, omega, time):
        """
        Request robot to rotate clockwise with specific time

        :param float omega: desired robot angular velocity in m/s
        :param float time: time for robot rotate clockwise
        """
        start_time = time.time()
        while time.time() - start_time < time:
            self.robot.set_mobile_base_speed_and_gripper_power(v=0, omega=-omega, gripper_power=0)
            time.sleep(0.05)
        print("Rotate clockwise action finish")

    def rotate_anticlockwise(self, omega, time):
        """
        Request robot to rotate anticlockwise with specific time

        :param float omega: desired robot angular velocity in m/s
        :param float time: time for robot rotate anticlockwise
        """
        start_time = time.time()
        while time.time() - start_time < time:
            self.robot.set_mobile_base_speed_and_gripper_power(v=0, omega=omega, gripper_power=0)
            time.sleep(0.05)
        print("Rotate anticlockwise action finish")
        


    def open_gripper(self):
        """
        Request robot arm open the gripper with specific time

        :param float power: desired power for robot arm to open the gripper
        :param float time: time for robot open the gripper
        """
        self.set_mobile_base_speed_and_gripper_power(v =0, omega=0, gripper_power=1.0,lapse=3. )
        print("Open gripper action finish")

    def close_gripper(self):
        """
        Request robot arm close the gripper with specific time

        :param float power: desired power for robot arm to close the gripper
        :param float time: time for robot close the gripper
        """
        self.set_mobile_base_speed_and_gripper_power(v=0, omega=0,gripper_power=-1.0, lapse=3.) 
        print("Close gripper action finish")

    def arm_forward(self):
        self.set_arm_pose(180, -60, lapse=6.)
        print("arm_forward finish")

    def arm_backword(self):
        self.set_arm_pose(60,40, lapse=6.)
        print("arm_backword finish")

    def stop_all(self):
        """
        Stop all the action of the robot
        """
        self.robot.set_mobile_base_speed_and_gripper_power(0., 0., 0.)
        print("Stop action finish")

    def led_control(self, r, g, b):
        """
        Request robot turn on the led with specific RGB coding

        :param float r: red level of robot led
        :param float g: green level of robot led
        :param float b: blue level of robot led
        """
        self.robot.set_leds(r, g, b)
        print("Set led action finish")

    def get_pose(self):
        """
        Get the robot, pickup, dropoff, obstacle position in XYZ plane
        """
        poses = self.robot.get_poses()
        print("Robot position (m): ", poses[0])
        print("Pickup position (m): ", poses[1])
        print("Dropoff position (m): ", poses[2])
        print("Obstacles 1 position (m): ", poses[3])
        print("Obstacles 2 position (m): ", poses[4])
        print("Obstacles 3 position (m): ", poses[5])

    def on_press(self, key):
        """
        listen to the keyboard event and control robot by WASD key, led by rgb key, and robot arm by arrow key

        :param enum key: keyboard press event
        """
        if key == Key.up:
            self.x += self.arm_position_step
            self.set_arm_pose(self.x, self.y)
            print("arm move forward")

        elif key == Key.down:
            self.x -= self.arm_position_step
            self.set_arm_pose(self.x, self.y)
            print("arm move backward")

        elif key == Key.left:
            self.y += self.arm_position_step
            self.set_arm_pose(self.x, self.y)
            print("arm move upward")

        elif key == Key.right:
            self.y -= self.arm_position_step
            self.set_arm_pose(self.x, self.y)
            print("arm move downward")

        elif key == Key.enter:
            self.x = 0
            self.y = 0
            self.set_arm_pose(self.x, self.y)
            print("arm back to origin")

        elif hasattr(key, 'char'):
            if key.char == "w":
                self.robot.set_mobile_base_speed_and_gripper_power(v=self.v, omega=0, gripper_power=0)
                print("robot move forward")

            elif key.char == "s":
                self.robot.set_mobile_base_speed_and_gripper_power(v=-self.v, omega=0, gripper_power=0)
                print("robot move backward")

            elif key.char == "a":
                self.robot.set_mobile_base_speed_and_gripper_power(v=0, omega=self.omega, gripper_power=0)
                print("robot rotate anti-clockwise")

            elif key.char == "d":
                self.robot.set_mobile_base_speed_and_gripper_power(v=0, omega=-self.omega, gripper_power=0)
                print("robot rotate clockwise")

            elif key.char == "r":
                self.led_control(255, 0, 0)
                print("turn on red led")

            elif key.char == "g":
                self.led_control(0, 255, 0)
                print("turn on green led")

            elif key.char == "b":
                self.led_control(0, 0, 255)
                print("turn on blue led")

            elif key.char == "+":
                self.gripper_power += self.power_step
                self.robot.set_mobile_base_speed_and_gripper_power(v=0, omega=0, gripper_power=self.gripper_power)
                print("open gripper")

            elif key.char == "-":
                self.gripper_power -= self.power_step
                self.robot.set_mobile_base_speed_and_gripper_power(v=0, omega=0, gripper_power=self.gripper_power)
                print("close gripper")

            elif key.char == "0":
                self.gripper_power = 0
                self.robot.set_mobile_base_speed_and_gripper_power(v=0, omega=0, gripper_power=self.gripper_power)
                print("gripper back to origin")

            else:
                print("Other key pressed")

        else:
            print("Other key pressed")
            
    def on_release(self, key):
        """
        listen to the keyboard event and stop the listener when press escape key

        :param enum key: keyboard release event
        """
        # Stop listener by pressing the escape key
        if key == Key.esc:
            # Stop listener
            return False
        
    def manual_control(self):
        """
        Manual control robot and arm movement and RGB led light
        """
        print("Start manual control function")

        # Collect events until released
        with Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join()