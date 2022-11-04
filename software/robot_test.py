# Code for testing whether all wheels move as intended.
import motion
import time

if __name__ == "__main__":
    robot = motion.OmniRobot()
    robot.open()
    thrower_speed = 0
    while True:
        robot.motor_test(speed=10, thrower=thrower_speed)
        time.sleep(0.1)
