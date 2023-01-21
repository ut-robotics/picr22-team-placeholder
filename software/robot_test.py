# Code for testing whether all wheels move as intended.
from modules.motion import OmniRobot
from modules.logger import Logger
from modules.helper import load_config
"""
   _____ _____ _____ _____ 
  |     |     |     |     |
  | %   | p   | l   | a   |
  |_____|_____|_____|_____|
  |     |     |     |     |
  | h   | o   | l   | d   |
  |_____|_____|_____|_____|
  |     |     |     |     |
  | e   | r   | c   | e   |
  |_____|_____|_____|_____|
  '); DROP TABLE BOT

"""
if __name__ == "__main__":
    config = load_config()
    # we're forcing this on so we can actually get feedback whether the motors work as expected
    config["logging"]["motor_speeds"] = True
    logger = Logger(config["logging"]["log_level"], name="%MOTOR-TESTER%")
    robot = OmniRobot(config=config, logger=logger)
    robot.open()
    thrower_speed = 0
    try:
        while True:
            robot.motor_test(speed=10, thrower=thrower_speed)
    except KeyboardInterrupt:
        robot.close()
        logger.log.info("Stopping...")
