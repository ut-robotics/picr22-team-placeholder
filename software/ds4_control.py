import motion
from helper import map_range
from pyPS4Controller.controller import Controller

# TODO - implement mode changing


class RobotDS4(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.robot = motion.OmniRobot()
        self.robot.open()
        self.max_speed = 1
        self.x_speed = 0
        self.y_speed = 0
        self.rot_speed = 0
        self.thrower_speed = 0  # speed currently
        self.thrower_speed_param = 50  # speed to use when we activate thrower
        print("Controller added.")

    def send_movement(self):
        self.robot.move(self.x_speed, self.y_speed, self.rot_speed,
                        self.thrower_speed, disableFailsafe=True)

    def axis_stop(self, axis):
        if axis == "x":
            self.x_speed = 0
        elif axis == "y":
            self.y_speed = 0
        elif axis == "rot":
            self.rot_speed = 0
        elif axis == "thrower":
            self.thrower_speed = 0
        else:
            print("Unknown axis", axis)

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
        self.axis_stop("x")

    def on_L1_press(self):
        print("L1 - left")
        self.x_speed = -self.max_speed
        self.send_movement()

    def on_L1_release(self):
        self.axis_stop("x")

    # Y-axis
    def on_R2_press(self, value):
        print("R2 - forwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, self.max_speed * 1000) / 1000
        self.send_movement()

    def on_R2_release(self):
        self.axis_stop("y")

    def on_L2_press(self, value):
        print("L2 - backwards")
        self.y_speed = map_range(value, -32767, 32767,
                                 0, -self.max_speed * 1000) / 1000
        self.send_movement()

    def on_L2_release(self):
        self.axis_stop("y")

    # TODO - figure out deadzones or go back to old system
    # Rotation
    def on_L3_left(self, value):
        print("L3 - rot. left")
        self.rot_speed = map_range(
            value, 0, -32767, 0, self.max_speed * 3000) / 1000
        self.send_movement()

    def on_L3_right(self, value):
        print("L3 - rot. right")
        self.rot_speed = map_range(
            value, 0, 32767, 0, -self.max_speed * 3000) / 1000
        self.send_movement()

    def on_L3_x_at_rest(self):
        self.axis_stop("rot")

    # MISC

    # switch modes
    def on_share_press(self):
        print("TODO - implement switching between autonomous and normal mode.")

    # quit
    def on_playstation_button_press(self):
        print("QUIT")
        self.robot.stop()
        self.robot.close()
        self.stop = True


if __name__ == "__main__":
    controller = RobotDS4(interface="/dev/input/js0",
                          connecting_using_ds4drv=False)
    controller.listen(timeout=60)
