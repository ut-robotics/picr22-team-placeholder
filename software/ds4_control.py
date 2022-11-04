from threading import Thread
import motion
from helper import map_range
from pyPS4Controller.controller import Controller
from enum import Enum


class Axis(Enum):
    """All possible movement axes"""
    X = 1
    Y = 2
    ROT = 3
    THROWER = 4


class RobotDS4Backend(Controller):
    def __init__(self, max_speed, analog_deadzone, default_thrower_speed, robot, **kwargs):
        """Initializes RobotDS4Backend

        Args:
            max_speed (float): Max speed for robot in metres
            analog_deadzone (int): Deadzone for analog stick
            default_thrower_speed (int): Default speed to use for thrower
            robot (OmniRobot): The robot to control
        """
        Controller.__init__(self, **kwargs)
        # if there is no connection with the robot, open one
        if robot == None:
            self.robot = motion.OmniRobot()
            self.robot.open()
        else:
            self.robot = robot
        # parameters
        self.max_speed = max_speed
        self.analog_deadzone = analog_deadzone
        # speed to use when we activate thrower
        self.thrower_speed_param = default_thrower_speed
        self.remote_controlled = False
        # default is stopped, 0
        self.x_speed = 0
        self.y_speed = 0
        self.rot_speed = 0
        self.thrower_speed = 0
        print("Controller added.")

    def send_movement(self):
        """Sends movement to the robot based on current speed values"""
        if self.remote_controlled:
            self.robot.move(self.x_speed, self.y_speed, self.rot_speed,
                            self.thrower_speed, disableFailsafe=True)

    def axis_stop(self, axis):
        """Stops a specified axis

        Args:
            axis (Axis): Axis to stop
        """
        print("Stopping axis", axis)
        if axis == Axis.X:
            self.x_speed = 0
        elif axis == Axis.Y:
            self.y_speed = 0
        elif axis == Axis.ROT:
            self.rot_speed = 0
        elif axis == Axis.THROWER:
            self.thrower_speed = 0
        else:
            print("Unknown axis", axis)
        self.send_movement()

    def on_up_arrow_press(self):
        """Increasing thrower speed"""
        self.thrower_speed_param += 1
        print("Increased thrower speed by 1, new speed:",
              self.thrower_speed_param)

    def on_down_arrow_press(self):
        """Decreasing thrower speed"""
        if self.thrower_speed_param == 49:
            print("Thrower - Already at minimum speed!")
        else:
            self.thrower_speed_param -= 1
            print("Decreased thrower speed by 1, new speed:",
                  self.thrower_speed_param)

    def on_triangle_press(self):
        """Activating thrower"""
        print("Triangle - Thrower")
        self.thrower_speed = self.thrower_speed_param
        self.send_movement()

    # TODO - might be broken, verify.
    def on_triangle_release(self):
        """Deactivating thrower"""
        print("Triangle - Thrower stop")
        self.axis_stop("thrower")
        self.send_movement()

    def on_R1_press(self):
        """Move right on the X-axis"""
        print("R1 - right")
        self.x_speed = self.max_speed
        self.send_movement()

    def on_R1_release(self):
        """Stop X-axis movement"""
        print("R1 release")
        self.axis_stop(Axis.X)

    def on_L1_press(self):
        """Move left on the X-axis"""
        print("L1 - left")
        self.x_speed = -self.max_speed
        self.send_movement()

    def on_L1_release(self):
        """Stop X-axis movement"""
        print("L1 release")
        self.axis_stop(Axis.X)

    def on_R2_press(self, value):
        """Move forwards on the Y-axis"""
        print("R2 - forwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, self.max_speed * 1000) / 1000
        self.send_movement()

    def on_R2_release(self):
        """Stop on the Y-axis"""
        self.axis_stop(Axis.Y)

    def on_L2_press(self, value):
        """Move backwards on the Y-axis"""
        print("L2 - backwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, -self.max_speed * 1000) / 1000
        self.send_movement()

    def on_L2_release(self):
        """Stop on the Y-axis"""
        self.axis_stop(Axis.Y)

    # Rotation
    def on_L3_left(self, value):
        """Rotate left"""
        print("L3 - rot. left")
        if value >= -self.analog_deadzone:
            self.axis_stop(Axis.ROT)
        else:
            self.rot_speed = map_range(
                value, 0, -32767, 0, self.max_speed * 3000) / 1000
            self.send_movement()

    def on_L3_right(self, value):
        """Rotate right"""
        print("L3 - rot. right")
        if value <= self.analog_deadzone:
            self.axis_stop(Axis.ROT)
        else:
            self.rot_speed = map_range(
                value, 0, 32767, 0, -self.max_speed * 3000) / 1000
            self.send_movement()

    def on_L3_x_at_rest(self):
        """Stop rotating"""
        self.axis_stop(Axis.ROT)

    # MISC

    # switch modes
    def on_share_press(self):
        """Toggle remote control"""
        self.robot.stop()
        self.remote_controlled = not self.remote_controlled
        print("Remote control:", self.remote_controlled)

    # quit
    def on_playstation_button_press(self):
        """Stop button"""
        self.stop_controller()

    def stop_controller(self):
        """Turn off the remote control."""
        print("QUIT")
        self.stop = True


class RobotDS4:
    """Class for enabling the use of a DualShock 4 controller for controlling the robot"""

    def __init__(self, max_speed=1, analog_deadzone=400, default_thrower_speed=50, robot=None):
        self.controller = None
        self.analog_deadzone = analog_deadzone
        self.max_speed = max_speed
        self.default_thrower_speed = default_thrower_speed
        self.robot = robot

    def start(self):
        """Starts controller listening in a separate thread"""
        self.thread = Thread(target=self.listen, args=()).start()

    def listen(self):
        """Listen for controller inputs"""
        self.controller = RobotDS4Backend(max_speed=self.max_speed, analog_deadzone=self.analog_deadzone,
                                          default_thrower_speed=self.default_thrower_speed, robot=self.robot, interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.controller.listen(timeout=60)

    def is_remote_controlled(self):
        """Whether the robot is currently remote controlled

        Returns:
            bool: is_remote_controlled
        """
        return self.controller.remote_controlled

    def is_stopped(self):
        """Whether the robot is stopped

        Returns:
            bool: is_stopped
        """
        return self.controller.stop

    def stop(self):
        """Stops the remote control."""
        self.controller.stop_controller()
