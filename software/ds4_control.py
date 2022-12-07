from threading import Thread
from helper import map_range, calculate_throw_speed
from pyPS4Controller.controller import Controller
from enum import Enum
from states import State
from pathlib import Path
from time import time, sleep


class Axis(Enum):
    """All possible movement axes"""
    X = 1
    Y = 2
    ROT = 3
    THROWER = 4


class ThrowMode(Enum):
    Assist = 0  # aim assist
    Manual = 1


class RobotDS4Backend(Controller):
    # speeds to use when we activate robot, these are changed when buttons are pressed
    # default is stopped, 0
    x_speed = 0
    y_speed = 0
    rot_speed = 0
    thrower_speed = 0
    thrower_active = False  # for toggling thrower
    throw_mode = ThrowMode.Manual

    def __init__(self, robot_data, **kwargs):
        """Initializes RobotDS4Backend

        Args:
            robot_data (Robot): The robot to control
        """
        Controller.__init__(self, **kwargs)
        self.robot_data = robot_data
        self.robot_data.logger.log.info("Controller added.")

        # debug function for thrower data collection
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
                                       self.thrower_speed, disable_failsafe=True)

    def axis_stop(self, axis):
        """Stops a specified axis

        Args:
            axis (Axis): Axis to stop
        """
        self.robot_data.logger.log.info(f"Stopping axis {axis}")
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
            self.robot_data.logger.log.warning("Already at max speed!")
        else:
            self.robot_data.manual_thrower_speed += 10
            self.robot_data.logger.log.warning(
                f"Increased thrower speed by 10, new speed: {self.robot_data.manual_thrower_speed}")
            if self.thrower_active:
                self.thrower_speed = self.robot_data.manual_thrower_speed
                self.send_movement()

    def on_down_arrow_press(self):
        """Decreasing thrower speed"""
        if self.robot_data.manual_thrower_speed <= 50:  # the actual limit is 49, but we can't get that with our current decrease interval anyways
            self.robot_data.logger.log.warning(
                "Thrower - Already at minimum speed!")
        else:
            self.robot_data.manual_thrower_speed -= 10
            self.robot_data.logger.log.warning(
                f"Decreased thrower speed by 10, new speed: {self.robot_data.manual_thrower_speed}")
        if self.thrower_active:
            self.thrower_speed = self.robot_data.manual_thrower_speed
            self.send_movement()

    def on_triangle_press(self):
        """Toggling thrower"""
        self.robot_data.logger.log.info("Triangle - Thrower")
        self.thrower_active = not self.thrower_active
        if self.thrower_active:
            self.thrower_speed = self.robot_data.manual_thrower_speed
        else:
            self.thrower_speed = 0
        self.send_movement()

    # X-axis
    def on_R3_left(self, value):
        """Move left on the X-axis"""
        self.robot_data.logger.log.info("R3 - drive left")
        if value >= self.robot_data.analog_deadzone:
            self.axis_stop(Axis.X)
        else:
            self.x_speed = map_range(
                value, 0, -32767, 0, -self.robot_data.max_speed * 1000) / 1000
            self.send_movement()

    def on_R3_right(self, value):
        """Move right on the X-axis"""
        self.robot_data.logger.log.info("R3 - drive right")
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
        self.robot_data.logger.log.info("R2 - forwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, self.robot_data.max_speed * 1000) / 1000
        self.send_movement()

    def on_R2_release(self):
        """Stop on the Y-axis"""
        self.axis_stop(Axis.Y)

    def on_L2_press(self, value):
        """Move backwards on the Y-axis"""
        self.robot_data.logger.log.info("L2 - backwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, -self.robot_data.max_speed * 1000) / 1000
        self.send_movement()

    def on_L2_release(self):
        """Stop on the Y-axis"""
        self.axis_stop(Axis.Y)

    # Rotation
    def on_L3_left(self, value):
        """Rotate left"""
        self.robot_data.logger.log.info("L3 - rot. left")
        if value >= -self.robot_data.analog_deadzone:
            self.axis_stop(Axis.ROT)
        else:
            self.rot_speed = map_range(
                value, 0, -32767, 0, self.robot_data.max_speed * 3000) / 1000
            self.send_movement()

    def on_L3_right(self, value):
        """Rotate right"""
        self.robot_data.logger.log.info("L3 - rot. right")
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
            """Save previous throw data."""
            if len(self.last_throw_data) > 0:
                with open(self.data_file, "a") as f:
                    f.write(self.last_throw_data)
                self.last_throw_data = str()
                self.robot_data.logger.log.info("Wrote speeds to file.")
            else:
                self.robot_data.logger.log.warning("Nothing to write!")

    def on_circle_press(self):
        """Attempt to throw the ball"""
        if self.robot_data.current_state == State.RemoteControl:
            # we can't get ball data if there's no ball and no point if robot can't find basket
            if self.robot_data.ball == None:
                self.robot_data.logger.log.warning("No ball found!")
                return
            elif self.robot_data.baskets[self.robot_data.basket_color].distance == -1:
                self.robot_data.logger.log.warning("No basket found!")
                return
            self.robot_data.logger.log.info("Saved speeds")
            self.last_throw_data = f"{self.robot_data.throw_move_speed},{self.robot_data.manual_thrower_speed},{self.robot_data.baskets[self.robot_data.basket_color].distance},{self.robot_data.ball.distance}\n"
            self.robot_data.throw_end_time = time() + self.robot_data.throw_time
            while time() < self.robot_data.throw_end_time:
                self.robot_data.logger.log.info(
                    f"--BallThrowRemote-- Throwing ball, basket distance: {self.robot_data.baskets[self.robot_data.basket_color].distance}")
                if self.throw_mode == ThrowMode.Manual:
                    self.robot_data.robot.move(0, self.robot_data.throw_move_speed,
                                               0, self.robot_data.manual_thrower_speed)
                    sleep(0.1)
                elif self.throw_mode == ThrowMode.Assist:
                    rot_delta = self.robot_data.middle_point - \
                        self.robot_data.baskets[self.robot_data.basket_color].x
                    rot_speed = -1 * rot_delta * 0.003
                    rot_sign = -1 if rot_speed >= 0 else 1
                    rot_speed = min(
                        abs(rot_speed), self.robot_data.max_speed) * rot_sign
                    self.robot_data.robot.move(0, self.robot_data.throw_move_speed,
                                               rot_speed, self.robot_data.manual_thrower_speed)

    def on_R1_press(self):
        """Adjust thrower speed based on basket distance"""
        if self.robot_data.current_state == State.RemoteControl:
            if self.robot_data.baskets[self.robot_data.basket_color].exists:
                self.robot_data.manual_thrower_speed = calculate_throw_speed(
                    self.robot_data.baskets[self.robot_data.basket_color].distance)
                self.robot_data.logger.log.info(
                    f"Adjusted thrower speed to {self.robot_data.manual_thrower_speed}")
                if self.thrower_active:
                    self.thrower_speed = self.robot_data.manual_thrower_speed
                    self.send_movement()
            else:
                self.robot_data.logger.log.warning(
                    "No basket visible, unable to adjust.")

    def on_L1_press(self):
        """Change basket throw mode"""
        if self.robot_data.current_state == State.RemoteControl:
            self.throw_mode = ThrowMode.Manual if self.throw_mode == ThrowMode.Assist else ThrowMode.Assist
            self.robot_data.logger.log.info(
                f"Current thrower mode is now {self.throw_mode}")

    # MISC
    # switch modes
    def on_share_press(self):
        """Toggle remote control"""
        self.robot_data.robot.stop()
        if self.robot_data.current_state != State.RemoteControl:
            self.robot_data.logger.log.info("Remote control: ON")
            self.robot_data.current_state = State.RemoteControl
        else:
            self.robot_data.logger.log.info("Remote control: OFF")

            self.robot_data.back_to_search_state()

    # quit
    def on_playstation_button_press(self):
        """Stop button"""
        self.stop_controller()

    def stop_controller(self):
        """Turn off the remote control."""
        self.robot_data.logger.log.info("QUIT")
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
