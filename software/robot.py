import motion
import time

if __name__ == "__main__":
    robot = motion.OmniRobot()
    robot.open()
    thrower_speed = 0
    while True: #mid, left, right
        robot.motor_test(thrower=thrower_speed)
        time.sleep(0.1)
