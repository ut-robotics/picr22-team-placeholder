# Code for testing whether all wheels move as intended.
import motion
from logger import Logger
from helper import load_config

if __name__ == "__main__":
    config = load_config()
    # we're forcing this on so we can actually get feedback whether the motors work as expected
    config["logging"]["motor_speeds"] = True
    logger = Logger(config["logging"]["log_level"], name="%MOTOR-TESTER%")
    robot = motion.OmniRobot(config=config, logger=logger)
    robot.open()
    thrower_speed = 0
    try:
        while True:
            robot.motor_test(speed=10, thrower=thrower_speed)
    except KeyboardInterrupt:
        robot.close()
        logger.log.info("Stopping...")
