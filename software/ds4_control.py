import motion
from pyPS4Controller.controller import Controller

# TODO - thrower


class RobotDS4(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.robot = motion.OmniRobot()
        self.robot.open()
        self.max_speed = 1
        self.x_speed = 0
        self.y_speed = 0
        self.rot_speed = 0
        self.thrower_speed = 0
        print("Controller added.")

    def send_movement(self):
        self.robot.move(self.x_speed, self.y_speed, self.rot_speed,
                        self.thrower_speed, disableFailsafe=True)

    # steering
    def on_left_arrow_press(self):
        print("Left - Move")
        self.x_speed = -self.max_speed
        self.send_movement()

    def on_right_arrow_press(self):
        print("Right - move")
        self.x_speed = self.max_speed
        self.send_movement()

    def on_left_right_arrow_release(self):
        print("L/R - stop")
        self.x_speed = 0
        self.send_movement()

    # triggers had duplicated signals, so I opted for bumpers for now
    def on_R1_press(self):
        print("R1 - forward")
        self.y_speed = self.max_speed
        self.send_movement()

    def on_R1_release(self):
        print("R1 - stop")
        self.y_speed = 0
        self.send_movement()

    def on_L1_press(self):
        print("L1 - reverse")
        self.y_speed = -self.max_speed
        self.send_movement()

    def on_L1_release(self):
        print("L1 - stop")
        self.y_speed = 0
        self.send_movement()

    def on_circle_press(self):
        print("Circle (L) - rot. left")
        self.rot_speed = -(3*self.max_speed)
        self.send_movement()

    def on_circle_release(self):
        print("Circle (L) - stop")
        self.rot_speed = 0
        self.send_movement()

    def on_square_press(self):
        print("Square (R) - rot. right")
        self.rot_speed = 3*self.max_speed
        self.send_movement()

    def on_square_release(self):
        print("Square (R) - stop")
        self.rot_speed = 0
        self.send_movement()

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
