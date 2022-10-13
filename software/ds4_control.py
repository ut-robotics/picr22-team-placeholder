import motion
from pyPS4Controller.controller import Controller

# TODO - thrower and figure out what values to actually feed the robot


class RobotDS4(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.robot = motion.OmniRobot()
        self.robot.open()
        self.speed = 0.5
        print("Controller connected.")

    # steering
    def on_left_arrow_press(self):
        print("Left - Move")
        self.robot.move(self.speed, 0, 0, 0, disableFailsafe=1)

    def on_right_arrow_press(self):
        print("Right - move")
        self.robot.move(-self.speed, 0, 0, 0, disableFailsafe=1)

    def on_left_right_arrow_release(self):
        self.robot.stop()
        print("Steering - stop")

    # triggers had duplicated signals, so I opted for bumpers for now
    def on_R1_press(self):
        self.robot.move(0, self.speed, 0, 0, disableFailsafe=1)
        print("R1 - forward")

    def on_R1_release(self):
        print("R1 - stop")
        self.robot.stop()

    def on_L1_press(self):
        print("L1 - reverse")
        self.robot.move(0, -self.speed, 0, 0, disableFailsafe=1)

    def on_L1_release(self):
        print("L1 - stop")
        self.robot.stop()

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
