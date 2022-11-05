# Code for testing whether all wheels move as intended.
import motion

if __name__ == "__main__":
    robot = motion.OmniRobot()
    robot.open()
    thrower_speed = 0
    try:
        while True:
            robot.motor_test(speed=10, thrower=thrower_speed)
    except KeyboardInterrupt:
        robot.close()
        print("Stopping...")