from threading import Thread
from helper import map_range, calculate_throw_speed
from pyPS4Controller.controller import Controller
from enum import Enum
from states import State
from pathlib import Path
import time
from Color import Color


class Axis(Enum):
    """All possible movement axes"""
    X = 1
    Y = 2
    ROT = 3
    THROWER = 4


class RobotDS4Backend(Controller):
    # speeds to use when we activate robot, these are changed when buttons are pressed
    # default is stopped, 0
    x_speed = 0
    y_speed = 0
    rot_speed = 0
    thrower_speed = 0
    thrower_active = False  # for toggling thrower

    def __init__(self, robot_data, **kwargs):
        """Initializes RobotDS4Backend

        Args:
            robot_data (Robot): The robot to control
        """
        Controller.__init__(self, **kwargs)
        self.robot_data = robot_data
        print("Controller added.")

        # debug function for thrower data collection
        if self.robot_data.debug_data_collection:
            self.last_throw_data = str()
            self.data_file = Path("data", "measurements.csv")
            self.data_file.parent.mkdir(parents=True, exist_ok=True)
            if not self.data_file.exists():
                with open(self.data_file, 'w') as f:
                    f.write("x speed,thrower speed,basket distance,ball distance\n")

    def send_movement(self):
        """Sends movement to the robot based on current speed values"""
        if self.robot_data.current_state == State.RemoteControl:
            self.robot_data.robot.move(self.x_speed, self.y_speed, self.rot_speed,
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
            raise ValueError(f"Invalid axis", axis)
        self.send_movement()

    # Thrower
    def on_up_arrow_press(self):
        """Increasing thrower speed"""
        if self.robot_data.manual_thrower_speed >= 2000:
            print("Already at max speed!")
        else:
            self.robot_data.manual_thrower_speed += 10
            print("Increased thrower speed by 10, new speed:",
                  self.robot_data.manual_thrower_speed)
            if self.thrower_active:
                self.thrower_speed = self.robot_data.manual_thrower_speed
                self.send_movement()

    def on_down_arrow_press(self):
        """Decreasing thrower speed"""
        if self.robot_data.manual_thrower_speed <= 50:  # the actual limit is 49, but we can't get that with our current decrease interval anyways
            print("Thrower - Already at minimum speed!")
        else:
            self.robot_data.manual_thrower_speed -= 10
            print("Decreased thrower speed by 10, new speed:",
                  self.robot_data.manual_thrower_speed)
        if self.thrower_active:
            self.thrower_speed = self.robot_data.manual_thrower_speed
            self.send_movement()

    def on_triangle_press(self):
        """Toggling thrower"""
        print("Triangle - Thrower")
        self.thrower_active = not self.thrower_active
        if self.thrower_active:
            self.thrower_speed = self.robot_data.manual_thrower_speed
        else:
            self.thrower_speed = 0
        self.send_movement()

    # X-axis
    def on_R3_left(self, value):
        """Move left on the X-axis"""
        print("R3 - drive left")
        if value >= self.robot_data.analog_deadzone:
            self.axis_stop(Axis.X)
        else:
            self.x_speed = map_range(
                value, 0, -32767, 0, -self.robot_data.max_speed * 1000) / 1000
            self.send_movement()

    def on_R3_right(self, value):
        """Move right on the X-axis"""
        print("R3 - drive right")
        if value <= -self.robot_data.analog_deadzone:
            self.axis_stop(Axis.X)
        else:
            self.x_speed = map_range(
                value, 0, 32767, 0, self.robot_data.max_speed * 1000) / 1000
            self.send_movement()

    def on_R3_x_at_rest(self):
        """Stop moving on the X-axis"""
        self.axis_stop(Axis.X)

    # Y-axis
    def on_R2_press(self, value):
        """Move forwards on the Y-axis"""
        print("R2 - forwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, self.robot_data.max_speed * 1000) / 1000
        self.send_movement()

    def on_R2_release(self):
        """Stop on the Y-axis"""
        self.axis_stop(Axis.Y)

    def on_L2_press(self, value):
        """Move backwards on the Y-axis"""
        print("L2 - backwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, -self.robot_data.max_speed * 1000) / 1000
        self.send_movement()

    def on_L2_release(self):
        """Stop on the Y-axis"""
        self.axis_stop(Axis.Y)

    # Rotation
    def on_L3_left(self, value):
        """Rotate left"""
        print("L3 - rot. left")
        if value >= -self.robot_data.analog_deadzone:
            self.axis_stop(Axis.ROT)
        else:
            self.rot_speed = map_range(
                value, 0, -32767, 0, self.robot_data.max_speed * 3000) / 1000
            self.send_movement()

    def on_L3_right(self, value):
        """Rotate right"""
        print("L3 - rot. right")
        if value <= self.robot_data.analog_deadzone:
            self.axis_stop(Axis.ROT)
        else:
            self.rot_speed = map_range(
                value, 0, 32767, 0, -self.robot_data.max_speed * 3000) / 1000
            self.send_movement()

    def on_L3_x_at_rest(self):
        """Stop rotating"""
        self.axis_stop(Axis.ROT)

    # DEBUG DATA COLLECTION
    def on_square_press(self):
        if self.robot_data.current_state == State.RemoteControl:
            if not self.robot_data.debug_data_collection:
                return
            """Save previous throw data."""
            if len(self.last_throw_data) > 0:
                with open(self.data_file, "a") as f:
                    f.write(self.last_throw_data)
                self.last_throw_data = str()
                print("Wrote speeds to file.")
            else:
                print("Nothing to write!")

    def on_circle_press(self):
        """Attempt to throw the ball"""
        if self.robot_data.current_state == State.RemoteControl:
            # we can't get ball data if there's no ball and no point if robot can't find basket
            if (self.robot_data.ball == None) or (self.robot_data.baskets[self.robot_data.basket_color].distance == -1):
                return
            print("Saved speeds")
            if self.robot_data.debug_data_collection:
                self.last_throw_data = f"{self.robot_data.throw_move_speed},{self.robot_data.manual_thrower_speed},{self.robot_data.baskets[self.robot_data.basket_color].distance},{self.robot_data.ball.distance}\n"
            for _ in range(3):
                print("--BallThrowRemote-- Throwing ball, basket distance:",
                      self.robot_data.baskets[self.robot_data.basket_color].distance)
                self.robot_data.robot.move(0, self.robot_data.throw_move_speed,
                                           0, self.robot_data.manual_thrower_speed)
                # this is blocking, but we're just gathering data anyways, so it's not much of an issue
                time.sleep(0.1)

    def on_R1_press(self):
        """Adjust thrower speed based on basket distance"""
        if self.robot_data.current_state == State.RemoteControl:
            if self.robot_data.baskets[self.robot_data.basket_color].exists:
                self.robot_data.manual_thrower_speed = calculate_throw_speed(
                    self.robot_data.baskets[self.robot_data.basket_color].distance)
                print("Adjusted thrower speed to",
                      self.robot_data.manual_thrower_speed)
                if self.thrower_active:
                    self.thrower_speed = self.robot_data.manual_thrower_speed
                    self.send_movement()
            else:
                print("No basket visible, unable to adjust.")

    def on_L1_press(self):
        """Change currently targeted basket"""
        if self.robot_data.current_state == State.RemoteControl:
            self.robot_data.basket_color = Color(2) if self.robot_data.basket_color == Color(3) else Color(3)
            self.robot_data.enemy_basket_color = Color(2) if self.robot_data.basket_color == Color(3) else Color(3)
            print("Current basket is now", self.robot_data.basket_color)

    # MISC
    # switch modes
    def on_share_press(self):
        """Toggle remote control"""
        self.robot_data.robot.stop()
        if self.robot_data.current_state != State.RemoteControl:
            print("Remote control: ON")
            self.robot_data.current_state = State.RemoteControl
        else:
            print("Remote control: OFF")
            
            self.robot_data.back_to_search_state()

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

    def __init__(self, robot_data):
        self.controller = None
        self.robot_data = robot_data

    def start(self):
        """Starts controller listening in a separate thread"""
        self.thread = Thread(target=self.listen, args=()).start()

    def listen(self):
        """Listen for controller inputs"""
        self.controller = RobotDS4Backend(
            robot_data=self.robot_data, interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.controller.listen(timeout=60)

    def is_stopped(self):
        """Whether the robot is stopped

        Returns:
            bool: is_stopped
        """
        return self.controller.stop

    def stop(self):
        """Stops the remote control."""
        self.controller.stop_controller()
