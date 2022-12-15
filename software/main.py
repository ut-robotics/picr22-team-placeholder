import image_processor
import camera
import motion
import cv2
from time import time
from ds4_control import RobotDS4
from Color import Color
from states import State, ThrowerState, SearchState, EscapeState, OrbitDirection
from helper import calculate_throw_speed, load_config
from referee import Referee
from logger import Logger
from random import choice
import numpy as np

# some objectives for the near future
# TODO - rework line detection
# TODO - make it work with TBD


class Robot:
    """The main class for %placeholder%"""

    # -- STATES --
    current_state = State.Stopped
    prev_state = None
    thrower_substate = ThrowerState.Off
    search_substate = SearchState.Off
    escape_substate = EscapeState.Off

    # -- FRAMECOUNTS --
    basket_too_close_frames = 0
    no_balls_frames = 0
    basket_no_basket_frames = 0

    # -- TIMEOUTS --
    throw_end_time = 0
    search_end_time = 0
    escape_state_end_time = 0
    orbit_start_time = 0
    orbit_direction_timeout = 0
    thrower_emergency_activation_time = 0  # to prevent the ball from getting stuck

    # -- BASKETS --
    opposite_basket = None  # for escapestate, we'll go towards this basket
    baskets = {Color.BLUE: None, Color.MAGENTA: None}
    basket_max_distance = 0
    enemy_basket_max_distance = 0
    basket_to_drive_to = None
    basket_distances = list()
    # this can be changed from referee command anyways, this is just a value to default to
    basket_color = Color.BLUE

    # -- BALLS --
    ball_count = 0
    ball = None
    last_seen_ball = None

    # -- DIRECTIONS --
    # to make the game more interesting, we'll randomly decide directions
    first_search_dir = choice([SearchState.Left, SearchState.Right])
    orbit_direction = choice([OrbitDirection.Left, OrbitDirection.Right])
    escape_direction = None

    # -- VARIA --
    thrower_speed = 0
    processed_data = None

    def __init__(self):
        # -- CONFIG --
        self.config = load_config()

        # -- LOGGING --
        self.logger = Logger(
            self.config["logging"]["log_level"], name="%placeholder%")

        # -- FPS counter --
        if self.config["logging"]["fps_counter"]:
            self.start = time()
            self.frame_cnt = 0

        # -- BALLS --
        if self.config["logging"]["ball_count"]:
            self.prev_ball_count = 0

        # -- DEBUG --
        self.debug = self.config["debug"]["debug"]

        # -- SEARCH --
        self.search_timeout = self.config["search"]["timeout"]
        self.enemy_basket_color = Color(
            2) if self.basket_color == Color(3) else Color(3)

        # -- ORBIT --
        self.orbit_dir_timeout_time = self.config["orbit"]["dir_timeout_time"]
        self.max_orbit_time = self.config["orbit"]["max_orbit_time"]

        # -- THROWER --
        # how long to stay in throw state
        self.throw_time = self.config["thrower"]["time"]
        # how far the ball has to be to prepare for throw, approx 10 cm
        self.min_distance = self.config["thrower"]["min_distance"]
        self.thrower_emergency_duration = self.config["thrower"]["emergency_duration"]
        self.thrower_emergency_interval = self.config["thrower"]["emergency_interval"]

        # -- MOVEMENT --
        self.max_speed = self.config["movement"]["max_speed"]
        self.throw_move_speed = self.config["movement"]["throw_move_speed"]
        self.search_speed = self.config["movement"]["search_speed"]
        # for escaping and searching, so we don't drive into the basket
        self.min_basket_dist = self.config["movement"]["min_basket_dist"]
        self.patrol_min_basket_dist = self.config["movement"]["patrol_min_basket_dist"]
        self.drive_to_ball_deadzone = self.config["movement"]["drive2ball_deadzone"]
        # this is the distance we use for driving to the ball, it's slightly shorter so the robot wouldn't drive into the ball at full speed most of the time
        self.drive_to_ball_minus_drive_dist = self.drive_to_ball_deadzone * 0.85
        if self.config["debug"]["fake_motion"]:
            self.robot = motion.FakeMotion(logger=self.logger, config=self.config)
        else:
            self.robot = motion.OmniRobot(logger=self.logger, config=self.config)
        self.robot.open()

        # -- CAMERA --
        self.cam = camera.RealsenseCamera(exposure=100)
        self.processor = image_processor.ImageProcessor(
            self.cam, logger=self.logger, debug=self.debug, config=self.config)
        self.processor.start()
        self.middle_point = self.cam.rgb_width // 2 + \
            self.config["camera"]["middle_offset"]
        self.camera_deadzone = self.config["camera"]["deadzone"]
        self.max_ball_miss = self.config["camera"]["max_ball_miss"]
        self.max_frames = self.config["camera"]["max_frames"]

        # -- CONTROLLER --
        self.analog_deadzone = self.config["controller"]["analog_deadzone"]
        self.manual_thrower_speed = self.config["controller"]["manual_thrower_speed"]
        self.controller = RobotDS4(robot_data=self)
        self.controller.start()

        # -- REFEREE --
        self.name = self.config["robot"]["name"]
        self.referee_ip = self.config["robot"]["referee_ip"]
        self.referee = Referee(robot_data=self)
        self.referee.start()

        # -- THE MAIN LOOP --
        self.main_loop()

    def back_to_search_state(self):
        """Returns to search state
        """
        self.current_state = State.Searching
        self.search_substate = SearchState.StartSearch

    def fps_counter(self):
        """Logs FPS"""
        self.frame_cnt += 1
        if self.frame_cnt % 30 == 0:
            self.frame_cnt = 0
            end = time()
            fps = 30 / (end - self.start)
            self.start = end
            self.logger.log.info(
                "FPS: {}, framecount: {}".format(fps, self.frame_cnt))

    def display_camera_feed(self):
        """Displays camera feed. NOTE: very performance intensive"""
        debug_frame = self.processed_data.debug_frame
        cv2.imshow('debug', debug_frame)
        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            raise KeyboardInterrupt

    def get_image_data(self):
        """Main non-state part of the loop"""
        # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
        if (self.current_state in [State.Stopped, State.EscapeFromBasket]) and (self.search_substate != SearchState.DriveToSearch): # don't use it when we don't need it, TODO - fix searching code enough so we wouldnt have to use depth there
            self.processed_data = self.processor.process_frame(aligned_depth=False)
        else:
            self.processed_data = self.processor.process_frame(aligned_depth=True)

        # -- BOILERPLATE CAMERA CODE, DO NOT TOUCH UNLESS REALLY NECESSARY --
        # FPS counter
        if self.config["logging"]["fps_counter"]:
            self.fps_counter()

        # Debug stuff, turn off when we don't need camera
        if self.debug:
            self.display_camera_feed()

        if self.prev_state != self.current_state:
            self.logger.log.info(f"CURRENT STATE - {self.current_state}")
            self.prev_state = self.current_state

        # -- REMOTE CONTROL STUFF --
        # stop button
        if self.controller.is_stopped():
            raise KeyboardInterrupt

        # -- AUTONOMOUS STUFF --
        self.ball_count = len(self.processed_data.balls)
        if self.ball_count == 0:
            self.no_balls_frames += 1
        else:
            self.no_balls_frames = 0

        if self.config["logging"]["ball_count"]:
            if self.prev_ball_count != self.ball_count:
                # this is for debugging
                self.logger.log.info("ball_count: {}".format(self.ball_count))
                self.prev_ball_count = self.ball_count

        # first element is always the closest ball
        if self.ball_count > 0:
            self.ball = self.processed_data.balls[0]
            self.last_seen_ball = self.ball
        elif self.no_balls_frames >= self.max_ball_miss:  # use old ball sometimes
            self.ball = None

        # TODO - implement code to prevent our robot from driving over lines
        self.baskets[Color.MAGENTA] = self.processed_data.basket_m
        self.baskets[Color.BLUE] = self.processed_data.basket_b

        if self.current_state not in [State.Stopped, State.RemoteControl, State.Debug, State.EscapeFromBasket]: #FIXME - search state will have very different basket distance values, figure something out
            if self.search_substate == SearchState.DriveToSearch:
                return
            for basket in self.baskets:
                if self.baskets[basket].exists:
                    self.basket_no_basket_frames = 0
                    if self.baskets[basket].distance < self.min_basket_dist:
                        if self.basket_too_close_frames > self.max_frames:
                            self.logger.log.error("BASKETING TIME")
                            self.opposite_basket = Color.MAGENTA if basket == Color.BLUE else Color.BLUE
                            self.current_state = State.EscapeFromBasket
                            self.escape_substate = EscapeState.StartEscape
                        else:
                            self.basket_too_close_frames += 1
                            self.logger.log.warning(
                                f"BASKET TOO CLOSE.... {self.baskets[basket].distance}")
                    else:
                        self.basket_too_close_frames = 0
                else:
                    self.basket_no_basket_frames += 1
                    if self.basket_no_basket_frames > self.max_frames:
                        self.logger.log.warning(f"no basket...")
                        self.basket_too_close_frames = 0

        if self.current_state not in [State.Orbiting, State.RemoteControl, State.Stopped]:
            # Randomize the direction to have a higher chance of actually getting the shorter way around, instead of always going left
            if time() > self.orbit_direction_timeout:
                self.logger.log.info(
                    "--ORBIT DIR TIMEOUT-- Randomizing orbit direction")
                self.orbit_direction = choice(
                    [OrbitDirection.Left, OrbitDirection.Right])
                self.orbit_direction_timeout = time() + self.orbit_dir_timeout_time
        else:
            self.orbit_direction_timeout = time() + self.orbit_dir_timeout_time

    def drive_to_object(self, object_x, object_dist):
        """Drive to an object

        Args:
            object_x (int): X coordinate of the object we want to drive to
            object_dist (int): Distance of the object we want to drive to
        """
        rot_delta = self.middle_point - object_x
        y_delta = self.min_distance - object_dist
        y_speed = -self.max_speed * y_delta * 0.005
        rot_speed = -1 * rot_delta * 0.003
        y_speed = np.clip(y_speed, -self.max_speed, self.max_speed)
        rot_speed = np.clip(rot_speed, -self.max_speed, self.max_speed) * -1
        thrower_speed = 0
        if y_speed < 0:
            # activate thrower to get balls out
            thrower_speed = 570
        elif time() > self.thrower_emergency_activation_time:
            if time() > self.thrower_emergency_activation_time + self.thrower_emergency_duration:
                self.logger.log.info(
                    f"--Drive2Object-- No thrower activation, time is {time()} but activation time is {self.thrower_emergency_activation_time + self.thrower_emergency_duration}")
                self.thrower_emergency_activation_time = time() + self.thrower_emergency_interval
            else:
                thrower_speed = 570
                self.logger.log.info("--Drive2Object-- Throwing!!")
        self.robot.move(0, y_speed, rot_speed, thrower_speed)

    def get_search_direction(self):
        if self.escape_direction != None:
            direction = self.escape_direction
            self.escape_direction = None
        elif self.last_seen_ball != None:
            if self.last_seen_ball.x >= self.middle_point:
                direction = SearchState.Right
            else:
                direction = SearchState.Left
        else:
            direction = self.first_search_dir  # this is randomly generated
        return direction

    def searching_state(self):
        """State for searching for the ball"""
        if self.ball_count != 0:
            self.logger.log.info(
                "--SEARCHING-- BALL FOUND, MOVING TO DRIVETOBALL")
            self.robot.stop()
            self.current_state = State.DriveToBall
            self.search_substate = SearchState.Off
            self.thrower_emergency_activation_time = time(
            ) + self.thrower_emergency_interval
            return

        if self.search_substate == SearchState.StartSearch:
            self.logger.log.info("StartSearch!!!")
            self.search_substate = self.get_search_direction()
            self.search_end_time = time() + self.search_timeout

        elif (time() > self.search_end_time) and (self.search_substate != SearchState.DriveToSearch):
            self.logger.log.warning(
                "--Searching-- Searched for too long, will drive to basket soon.")
            if self.basket_to_drive_to == None:
                if self.enemy_basket_max_distance >= self.basket_max_distance:
                    self.basket_to_drive_to = self.enemy_basket_color
                else:
                    self.basket_to_drive_to = self.basket_color
            if self.baskets[self.basket_to_drive_to].exists:
                self.robot.stop()
                self.search_substate = SearchState.DriveToSearch
                self.thrower_emergency_activation_time = time(
                ) + self.thrower_emergency_interval

        # TODO - maybe check lines and turn 45 degrees when hitting a line

        if self.search_substate != SearchState.DriveToSearch:
            if self.baskets[self.basket_color].exists:
                if self.baskets[self.basket_color].distance < self.enemy_basket_max_distance:
                    self.basket_max_distance = self.baskets[self.basket_color].distance
            elif self.baskets[self.enemy_basket_color].exists:
                if self.baskets[self.enemy_basket_color].distance < self.enemy_basket_max_distance:
                    self.enemy_basket_max_distance = self.baskets[self.enemy_basket_color].distance

        if self.search_substate == SearchState.Left:
            self.logger.log.info("--Searching-- Moving LEFT to look for ball")
            self.robot.move(0, 0, self.search_speed, 0)
        elif self.search_substate == SearchState.Right:
            self.logger.log.info("--Searching-- Moving RIGHT to look for ball")
            self.robot.move(0, 0, -self.search_speed, 0)

        elif self.search_substate == SearchState.DriveToSearch:
            if self.baskets[self.basket_to_drive_to].exists:
                ideal_distance = self.baskets[self.basket_to_drive_to].distance - (
                    self.patrol_min_basket_dist / 1.5)

                if not self.patrol_min_basket_dist > self.baskets[self.basket_to_drive_to].distance:
                    self.basket_too_close_frames = 0
                    self.drive_to_object(
                        self.baskets[self.basket_to_drive_to].x, ideal_distance)
                    self.logger.log.info(
                        f"--Searching-- Drive2Search: Basket Dist: {self.baskets[self.basket_to_drive_to].distance}, ideal distance: {ideal_distance}")
                elif self.basket_too_close_frames >= self.max_frames:
                    self.logger.log.warning(
                        f"--SEARCHING-- Basket too close, distance: {self.baskets[self.basket_to_drive_to].distance}, back to rotating!")
                    self.basket_to_drive_to = None
                    self.basket_max_distance = 0
                    self.enemy_basket_max_distance = 0
                    self.back_to_search_state()
                else:
                    self.basket_too_close_frames += 1
            else:
                self.logger.log.warning(
                    "--Searching-- Drive2Search: Basket does not exist, going back to rotating!")
                self.basket_to_drive_to = None
                self.basket_max_distance = 0
                self.enemy_basket_max_distance = 0
                self.back_to_search_state()

    def drive_to_ball_state(self):
        """State for driving to the ball."""
        if self.no_balls_frames >= self.max_ball_miss:  # lost the ball
            self.logger.log.info(
                f"--Drive2Ball-- Haven't seen ball for {self.max_ball_miss} frames, going back to search.")
            self.back_to_search_state()
            return
        self.logger.log.info("--Drive2Ball-- Driving to the ball.")

        # TODO - either rework this or expose constants in config
        if not self.min_distance + 35 > self.ball.distance > self.min_distance - 35:
            self.drive_to_object(self.ball.x, self.ball.distance - 30)
            self.logger.log.info(
                f"--Drive2Ball-- Ball distance {self.ball.distance}.")
        else:
            self.robot.stop()
            self.current_state = State.Orbiting
            self.orbit_start_time = time()

    def orbiting_state(self):
        """State for orbiting around the ball and trying to find the basket."""
        if time() > self.orbit_start_time + self.max_orbit_time:  # TODO - rethink how we timeout, the robot will just find the same ball again instantly
            self.logger.log.info(
                "--ORBIT-- Orbiting for too long, going back to search")
            self.robot.stop()
            self.back_to_search_state()
            return

        if self.no_balls_frames >= self.max_ball_miss:  # lost the ball
            self.logger.log.info(
                f"--Orbiting-- Haven't seen ball for {self.max_ball_miss} frames, going back to search.")
            self.back_to_search_state()
            return

        if self.ball.distance > 3 * self.min_distance:
            self.back_to_search_state()
            return

        # NOTE - adjust these values to improve orbiting
        x_delta = self.middle_point - self.ball.x
        x_speed = self.orbit_direction * 0.45
        y_delta = self.min_distance - self.ball.distance
        y_speed = -1 * y_delta * 0.005

        # TODO - adjust this further, not sure if too much or too little currently
        rot_speed = 6 * (x_delta / self.middle_point)

        self.logger.log.info(
            f"--Orbiting-- Ball X {self.ball.x} Ball X delta {x_delta}")

        if self.baskets[self.basket_color].exists:
            x_speed = -1 * x_delta * 0.0018
            basket_delta = self.baskets[self.basket_color].x - \
                self.middle_point

            self.logger.log.info(f"--Orbiting-- Basket delta {basket_delta}")

            rot_speed = -1 * basket_delta * 0.009

            if abs(x_delta) <= 12 and abs(basket_delta) <= 15:
                self.robot.stop()
                self.current_state = State.BallThrow
                self.thrower_substate = ThrowerState.StartThrow
                self.throw_end_time = time() + self.throw_time
                return

        # Clamp the x_speed and y_speed values
        x_speed = np.clip(x_speed, -self.max_speed, self.max_speed)
        y_speed = np.clip(y_speed, -self.max_speed, self.max_speed)
        # TODO - clamp rot speed, it goes crazy when it sees multiple balls nearby
        # max_rot_speed = ???
        # rot_speed = np.clip(rot_speed, -max_rot_speed, max_rot_speed)
        self.logger.log.info(
            f"--Orbiting-- MoveX {x_speed} MoveY {y_speed} rot {rot_speed}")

        self.robot.move(x_speed, y_speed, rot_speed)

    def ball_throw_state(self):
        """State for throwing the ball into the basket"""
        if time() > self.throw_end_time:  # end the throw after a specified amount of time
            self.thrower_substate = ThrowerState.EndThrow

        if self.thrower_substate == ThrowerState.StartThrow:
            if self.baskets[self.basket_color].exists:
                self.basket_distances = list()
                # all our data is from slightly away from the ball, so always adjusting the speed is a bad idea
                self.thrower_speed = calculate_throw_speed(
                    self.baskets[self.basket_color].distance)
                self.thrower_substate = ThrowerState.MidThrow
                self.logger.log.info(
                    f"--BallThrow-- Starting throw, basket distance: {self.baskets[self.basket_color].distance}, speed: {self.thrower_speed}")
                self.robot.move(0, self.throw_move_speed,
                                0, self.thrower_speed)
                self.basket_distances.append(
                    self.baskets[self.basket_color].distance)
            else:
                self.logger.log.warning(
                    "--BallThrow-- Trying to start throw, but there is no basket!")
        elif self.thrower_substate == ThrowerState.MidThrow:
            if len(self.basket_distances) < 5:
                self.basket_distances.append(
                    self.baskets[self.basket_color].distance)
                self.thrower_speed = calculate_throw_speed(
                    np.mean(self.basket_distances))
            rot_delta = self.middle_point - self.baskets[self.basket_color].x
            rot_speed = -1 * rot_delta * 0.003
            rot_sign = -1 if rot_speed >= 0 else 1
            rot_speed = min(abs(rot_speed), self.max_speed) * rot_sign
            self.logger.log.info(
                f"--BallThrow-- Throwing, current basket distance {self.baskets[self.basket_color].distance}.")
            self.robot.move(0, self.throw_move_speed,
                            rot_speed, self.thrower_speed)
        elif self.thrower_substate == ThrowerState.EndThrow:
            self.logger.log.info(
                f"--BallThrow-- Finishing throw, current basket distance {self.baskets[self.basket_color].distance}.")
            self.back_to_search_state()
            self.thrower_substate = ThrowerState.Off
            self.thrower_speed = 0
            self.basket_distances = list()

    def parse_referee_cmd(self):
        """Function for parsing referee commands"""
        cmd = self.referee.get_cmd()
        if cmd == None:
            return

        if self.name in cmd["targets"]:
            robot_index = cmd["targets"].index(self.name)
            if robot_index > 1:
                return  # this should never happen
        else:
            robot_index = None
            return  # we filter it in referee already, so this shouldnt happen, but better safe than sorry

        if cmd["signal"] == "start":
            basket_str = cmd["baskets"][robot_index]
            if basket_str == "blue":
                self.basket_color = Color.BLUE
            elif basket_str == "magenta":
                self.basket_color = Color.MAGENTA
            else:
                raise ValueError(f"Unknown basket colour: {basket_str}")

            self.enemy_basket_color = Color(
                2) if self.basket_color == Color(3) else Color(3)
            self.back_to_search_state()
            opponent_index = 0 if robot_index == 1 else 1
            if len(cmd["targets"]) > 1:
                self.logger.log.info(
                    f"STARTING ROBOT, basket: {self.basket_color}, opponent: {cmd['targets'][opponent_index]}")
            else:
                self.logger.log.info(
                    f"STARTING ROBOT, basket: {self.basket_color}, no opponent.")

        elif cmd["signal"] == "stop":
            self.logger.log.info("STOPPING ROBOT")
            self.current_state = State.Stopped
        else:
            raise ValueError(f'Unknown signal: {cmd["signal"]}')

    def stop_state(self):
        """State for stopping."""
        self.robot.move(0, 0, 0, 0)

    def escape_from_basket_state(self):
        """State for escaping in case we get too close to the basket"""
        if self.escape_substate == EscapeState.StartEscape:
            self.escape_substate = EscapeState.Reverse
            self.escape_state_end_time = time() + 0.7  # dont reverse for too long
            self.escape_direction = self.get_search_direction()
        elif self.escape_substate == EscapeState.Reverse:
            self.logger.log.info("--ESCAPE-- Reversing")
            if time() > self.escape_state_end_time:
                self.escape_substate = EscapeState.TurningFromBasket
                # rotate for 2 seconds, or until opposite basket is found
                self.escape_state_end_time = time() + 2
            else:
                # TODO - might need to adjust for new robot
                self.robot.move(0, -self.max_speed * 0.75, 0)
        elif self.escape_substate == EscapeState.TurningFromBasket:
            self.logger.log.info("--ESCAPE-- Turning from basket")
            if (self.baskets[self.opposite_basket].exists):
                self.robot.stop()
                self.escape_substate = EscapeState.DrivingAway
                self.escape_state_end_time = time() + 1.5
            elif time() > self.escape_state_end_time:
                self.escape_substate = EscapeState.Off
                self.escape_state_end_time = 0
                self.back_to_search_state()  # go back to searching and hope for the best
            else:
                if self.escape_direction == SearchState.Left:
                    self.robot.move(0, 0, self.search_speed)
                elif self.escape_direction == SearchState.Right:
                    self.robot.move(0, 0, -self.search_speed)
                else:
                    raise ValueError(self.escape_direction)
        elif self.escape_substate == EscapeState.DrivingAway:
            self.logger.log.info("--ESCAPE-- Driving away")
            if time() > self.escape_state_end_time:
                self.escape_substate = EscapeState.Off
                self.escape_state_end_time = 0
                self.back_to_search_state()  # go back to searching and hope for the best
            else:
                # TODO - might need to adjust for new robot
                self.robot.move(0, self.max_speed*0.75, 0)

    def main_loop(self):
        """The main loop for our funny bot, where we decide what to do"""
        try:
            while True:
                self.parse_referee_cmd()
                self.get_image_data()

                if self.current_state == State.RemoteControl:
                    continue

                elif self.current_state == State.Stopped:
                    self.stop_state()

                elif self.current_state == State.Searching:
                    self.searching_state()

                elif self.current_state == State.Orbiting:
                    self.orbiting_state()

                elif self.current_state == State.DriveToBall:
                    self.drive_to_ball_state()

                elif self.current_state == State.BallThrow:
                    self.ball_throw_state()

                elif self.current_state == State.EscapeFromBasket:
                    self.escape_from_basket_state()

        except KeyboardInterrupt:
            self.logger.log.info("Closing....")
        except:
            self.logger.log.exception('')
        finally:
            self.robot.close()
            self.controller.stop()
            cv2.destroyAllWindows()
            self.processor.stop()
            self.referee.close()


if __name__ == "__main__":
    robot = Robot()
