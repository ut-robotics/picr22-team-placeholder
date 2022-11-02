from threading import Thread
import motion
from helper import map_range
from pyPS4Controller.controller import Controller
from enum import Enum


class Axis(Enum):
    X = 1
    Y = 2
    ROT = 3
    THROWER = 4


class RobotDS4Backend(Controller):
    def __init__(self, max_speed, analog_deadzone, default_thrower_speed, **kwargs):
        Controller.__init__(self, **kwargs)
        self.robot = motion.OmniRobot()
        self.robot.open()
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
        if self.remote_controlled:
            self.robot.move(self.x_speed, self.y_speed, self.rot_speed,
                            self.thrower_speed, disableFailsafe=True)

    def axis_stop(self, axis):
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

    # thrower
    def on_up_arrow_press(self):
        self.thrower_speed_param += 1
        print("Increased thrower speed by 1, new speed:",
              self.thrower_speed_param)

    def on_down_arrow_press(self):
        if self.thrower_speed_param == 49:
            print("Thrower - Already at minimum speed!")
        else:
            self.thrower_speed_param -= 1
            print("Decreased thrower speed by 1, new speed:",
                  self.thrower_speed_param)

    def on_triangle_press(self):
        print("Triangle - Thrower")
        self.thrower_speed = self.thrower_speed_param
        self.send_movement()

    def on_triangle_release(self):
        print("Triangle - Thrower stop")
        self.axis_stop("thrower")
        self.send_movement()

    # X-axis
    def on_R1_press(self):
        print("R1 - right")
        self.x_speed = self.max_speed
        self.send_movement()

    def on_R1_release(self):
        print("R1 release")
        self.axis_stop(Axis.X)

    def on_L1_press(self):
        print("L1 - left")
        self.x_speed = -self.max_speed
        self.send_movement()

    def on_L1_release(self):
        print("L1 release")
        self.axis_stop(Axis.X)

    # Y-axis
    def on_R2_press(self, value):
        print("R2 - forwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, self.max_speed * 1000) / 1000
        self.send_movement()

    def on_R2_release(self):
        self.axis_stop(Axis.Y)

    def on_L2_press(self, value):
        print("L2 - backwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, -self.max_speed * 1000) / 1000
        self.send_movement()

    def on_L2_release(self):
        self.axis_stop(Axis.Y)

    # Rotation
    def on_L3_left(self, value):
        print("L3 - rot. left")
        if value >= -self.analog_deadzone:
            self.axis_stop(Axis.ROT)
        else:
            self.rot_speed = map_range(
                value, 0, -32767, 0, self.max_speed * 3000) / 1000
            self.send_movement()

    def on_L3_right(self, value):
        print("L3 - rot. right")
        if value <= self.analog_deadzone:
            self.axis_stop(Axis.ROT)
        else:
            self.rot_speed = map_range(
                value, 0, 32767, 0, -self.max_speed * 3000) / 1000
            self.send_movement()

    def on_L3_x_at_rest(self):
        self.axis_stop(Axis.ROT)

    # MISC

    # switch modes
    def on_share_press(self):
        self.robot.stop()
        self.remote_controlled = not self.remote_controlled
        print("Remote control:", self.remote_controlled)

    # quit
    def on_playstation_button_press(self):
        print("QUIT")
        self.robot.stop()
        self.robot.close()
        self.stop = True


class RobotDS4:
    def __init__(self, max_speed=1, analog_deadzone=400, default_thrower_speed=50):
        self.controller = None
        self.analog_deadzone = analog_deadzone
        self.max_speed = max_speed
        self.default_thrower_speed = default_thrower_speed

    def start(self):
        Thread(target=self.listen, args=()).start()

    def listen(self):
        self.controller = RobotDS4Backend(max_speed=self.max_speed, analog_deadzone=self.analog_deadzone,
                                          default_thrower_speed=self.default_thrower_speed, interface="/dev/input/js0", connecting_using_ds4drv=False)
        self.controller.listen(timeout=60)

    def is_remote_controlled(self):
        return self.controller.remote_controlled

    def is_stopped(self):
        return self.controller.stop


if __name__ == "__main__":
    controller = RobotDS4()
    controller.start()
