import image_processor
import camera
import motion
import cv2
from time import time
from ds4_control import RobotDS4
from Color import Color
from states import State, ThrowerState, SearchState, EscapeState
from helper import calculate_throw_speed
from referee import Referee
from logger import Logger

class Robot:
    """The main class for %placeholder%"""
    current_state = State.Stopped
    thrower_substate = ThrowerState.Off
    thrower_speed = 0
    search_substate = SearchState.Off
    escape_substate = EscapeState.Off
    basket_too_close_frames = 0
    opposite_basket = None  # for escapestate, we'll go towards this basket

    # FPS counter
    start = time()
    frame_cnt = 0


    # probably not needed, just for debugging right now to cut down on log spam
    prev_ball_count = 0
    no_balls_frames = 0
    orbit_start = 0
    throw_end_time = 0
    processed_data = 0
    ball_count = 0
    ball = None
    baskets = {Color.BLUE: None, Color.MAGENTA: None}
    basket_max_distance = 0
    enemy_basket_max_distance = 0
    search_end_time = 0
    last_seen_ball = None
    basket_to_drive_to = None
    escape_state_end = 0

    def __init__(self,
                 debug: bool,
                 camera_deadzone: int,
                 max_speed: float,
                 search_speed: float,
                 throw_time: float,
                 min_distance: int,
                 max_ball_miss: int,
                 use_realsense: bool,
                 middle_offset: int,
                 basket_color: Color,
                 max_orbit_time: int,
                 manual_thrower_speed: int,
                 analog_deadzone: int,
                 debug_data_collection: bool,
                 throw_move_speed: float,
                 referee_ip: str,
                 name: str,
                 search_timeout: int,
                 search_min_basket_dist: int,
                 avg_fps: int):
        # initialize logging
        self.logger = Logger()
        self.robot = motion.OmniRobot(robot_data=self)
        self.debug = debug
        self.debug_data_collection = debug_data_collection
        if use_realsense:
            # camera instance for realsense cameras
            self.cam = camera.RealsenseCamera(exposure=100)
        else:
            # camera instance for normal web cameras
            self.cam = camera.OpenCVCamera(id=2)
        self.processor = image_processor.ImageProcessor(self.cam, debug=debug)
        self.processor.start()
        self.middle_point = self.cam.rgb_width // 2 + middle_offset
        self.camera_deadzone = camera_deadzone

        self.robot.open()
        self.analog_deadzone = analog_deadzone
        self.manual_thrower_speed = manual_thrower_speed

        self.max_speed = max_speed
        self.throw_move_speed = throw_move_speed
        self.search_speed = search_speed
        self.throw_time = throw_time  # how long to stay in throw state
        # how far the ball has to be to prepare for throw, approx 10 cm
        self.min_distance = min_distance

        self.max_ball_miss = max_ball_miss

        self.basket_color = basket_color
        self.enemy_basket_color = Color(
            2) if self.basket_color == Color(3) else Color(3)
        self.max_orbit_time = max_orbit_time
        self.avg_fps = avg_fps
        self.search_timeout = search_timeout
        self.search_min_basket_dist = search_min_basket_dist
        self.controller = RobotDS4(robot_data=self)
        self.controller.start()
        self.referee_ip = referee_ip
        self.name = name
        self.referee = Referee(robot_data=self)
        self.referee.start()
        self.main_loop()

    def back_to_search_state(self):
        """Returns to search state
        """
        self.current_state = State.Searching
        self.search_substate = SearchState.StartSearch

    def fps_counter(self):
        """Prints FPS"""
        self.frame_cnt += 1
        if self.frame_cnt % 30 == 0:
            self.frame_cnt = 0
            end = time()
            fps = 30 / (end - self.start)
            self.start = end
            self.logger.log.info("FPS: {}, framecount: {}".format(fps, self.frame_cnt))

    def display_camera_feed(self):
        debug_frame = self.processed_data.debug_frame
        cv2.imshow('debug', debug_frame)
        k = cv2.waitKey(1) & 0xff
        if k == ord('q'):
            raise KeyboardInterrupt

    def get_image_data(self):
        """Main non-state part of the loop"""
        # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
        self.processed_data = self.processor.process_frame(aligned_depth=True)

        # -- BOILERPLATE CAMERA CODE, DO NOT TOUCH UNLESS REALLY NECESSARY --
        # FPS counter
        self.fps_counter()

        # Debug stuff, turn off when we don't need camera
        if self.debug:
            self.display_camera_feed()

        #self.logger.log.info("CURRENT STATE -", self.current_state)

        # -- REMOTE CONTROL STUFF --
        # stop button
        if self.controller.is_stopped():
            raise KeyboardInterrupt

        # verify if robot is remote controlled, no autonomous stuff if robot is remote controlled
        if not self.debug_data_collection:  # data collection requires camera
            if self.current_state == State.RemoteControl:
                return

        # -- AUTONOMOUS STUFF --
        self.ball_count = len(self.processed_data.balls)
        if self.ball_count == 0:
            self.no_balls_frames += 1
        else:
            self.no_balls_frames = 0

        if self.prev_ball_count != self.ball_count:
            # this is for debugging
            #self.logger.log.info("ball_count: {}".format(self.ball_count))
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

        if self.current_state not in [State.Stopped, State.RemoteControl, State.Debug]:
            if self.search_substate == SearchState.DriveToSearch:
                return
            for basket in self.baskets:
                if self.baskets[basket].exists:
                    if self.baskets[basket].distance < 400:
                        if self.basket_too_close_frames > self.avg_fps:
                            self.opposite_basket = Color.MAGENTA if basket == Color.BLUE else Color.BLUE
                            self.current_state = State.EscapeFromBasket
                            self.escape_substate = EscapeState.StartEscape
                        else:
                            self.basket_too_close_frames += 1
                    else:
                        self.basket_too_close_frames = 0

    def searching_state(self):
        """State for searching for the ball"""
        if self.ball_count != 0: 
            self.logger.log.info("--SEARCHING-- BALL FOUND, MOVING TO DRIVE2BALL")
            self.robot.stop()
            self.current_state = State.DriveToBall
            self.search_substate = SearchState.Off
            return

        if self.search_substate == SearchState.StartSearch:
            self.logger.log.info("StartSearch!!!")
            if self.last_seen_ball != None:
                if self.last_seen_ball.x >= self.middle_point:
                    self.search_substate = SearchState.Right
                else:
                    self.search_substate = SearchState.Left
            else:
                self.search_substate = SearchState.Left
            self.search_end_time = time() + self.search_timeout

        elif (time() > self.search_end_time) and (self.search_substate != SearchState.DriveToSearch):
            self.logger.log.warning("--Searching-- Searched for too long, will drive to basket soon.")
            if self.basket_to_drive_to == None:
                if self.enemy_basket_max_distance >= self.basket_max_distance:
                    self.basket_to_drive_to = self.enemy_basket_color
                else:
                    self.basket_to_drive_to = self.basket_color
            if self.baskets[self.basket_to_drive_to].exists:
                self.robot.stop()
                self.search_substate = SearchState.DriveToSearch
            else:
                self.logger.log.info("basket does not exist, basket distances are", self.enemy_basket_max_distance, self.basket_max_distance,
                      self.baskets[self.basket_color].exists, self.baskets[self.enemy_basket_color].exists, self.baskets[self.basket_to_drive_to])

        # TODO - maybe check lines and turn 45 degrees when hitting a line

        if self.search_substate != SearchState.DriveToSearch:
            if self.baskets[self.basket_color].exists:
                if self.baskets[self.basket_color].distance > self.enemy_basket_max_distance:
                    self.basket_max_distance = self.baskets[self.basket_color].distance
            elif self.baskets[self.enemy_basket_color].exists:
                if self.baskets[self.enemy_basket_color].distance > self.enemy_basket_max_distance:
                    self.enemy_basket_max_distance = self.baskets[self.enemy_basket_color].distance

        if self.search_substate == SearchState.Left:
            self.logger.log.info("--Searching-- Moving LEFT to look for ball")
            self.robot.move(0, 0, self.search_speed, 0)
        elif self.search_substate == SearchState.Right:
            self.logger.log.info("--Searching-- Moving RIGHT to look for ball")
            self.robot.move(0, 0, -self.search_speed, 0)

        elif self.search_substate == SearchState.DriveToSearch: # TODO - we're not checking if the basket exists
            if not self.search_min_basket_dist > self.baskets[self.basket_to_drive_to].distance:
                self.basket_too_close_frames = 0
                rot_delta = self.middle_point - \
                    self.baskets[self.basket_to_drive_to].x
                y_delta = self.min_distance - \
                    self.baskets[self.basket_to_drive_to].distance

                y_speed = -self.max_speed * y_delta * 0.0006
                rot_speed = -1 * rot_delta * 0.003

                y_sign = 1 if y_speed >= 0 else -1
                rot_sign = -1 if rot_speed >= 0 else 1

                y_speed = min(abs(y_speed), self.max_speed) * y_sign
                rot_speed = min(abs(rot_speed), self.max_speed) * rot_sign
                self.logger.log.info(
                    f"--Searching-- Drive2Search: Basket Dist: {self.baskets[self.basket_to_drive_to].distance}, y_speed {y_speed}, rot_speed {rot_speed}")
                self.robot.move(0, y_speed, rot_speed)
            elif self.basket_too_close_frames >= self.avg_fps:
                self.logger.log.warning(
                    f"--SEARCHING-- Basket too close, distance: {self.baskets[self.basket_to_drive_to].distance}, back to rotating!")
                self.basket_to_drive_to = None
                self.basket_max_distance = 0
                self.enemy_basket_max_distance = 0
                self.back_to_search_state()
            else:
                self.basket_too_close_frames += 1

    def drive_to_ball_state(self):
        """State for driving to the ball."""
        if self.no_balls_frames >= self.max_ball_miss:  # lost the ball
            self.logger.log.info(
                f"--DriveToBall-- Haven't seen ball for {self.max_ball_miss} frames, going back to search.")
            self.back_to_search_state()
            return
        self.logger.log.info("--DriveToBall-- Driving to ball.")

        if not self.min_distance + 35 > self.ball.distance > self.min_distance - 35:
            rot_delta = self.middle_point - self.ball.x
            y_delta = self.min_distance - self.ball.distance

            y_speed = -self.max_speed * y_delta * 0.01
            rot_speed = -self.max_speed * rot_delta * 0.003

            y_sign = 1 if y_speed >= 0 else -1
            rot_sign = -1 if rot_speed >= 0 else 1

            y_speed = min(abs(y_speed), self.max_speed) * y_sign
            rot_speed = min(abs(rot_speed), self.max_speed) * rot_sign
            self.logger.log.info(
                f"--DriveToBall-- ball distance {self.ball.distance}, y_speed {y_speed}, rot_speed {rot_speed}")
            self.robot.move(0, y_speed, rot_speed)
        else:
            self.robot.stop()
            self.current_state = State.Orbiting
            self.orbit_start = time()

    def orbiting_state(self):
        """State for orbiting around the ball and trying to find the basket."""
        if time() > self.orbit_start + self.max_orbit_time:
            self.logger.log.info("--ORBIT-- Orbiting for too long, going back to search")
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

        # TODO - adjust these values to improve orbiting
        x_delta = self.middle_point - self.ball.x
        x_speed = -1 * x_delta * 0.0018

        y_delta = self.min_distance - self.ball.distance
        y_speed = -1 * y_delta * 0.001

        rot_speed = self.max_speed * 0.7

        self.logger.log.info(f"--Orbiting-- Ball X {self.ball.x} Ball X delta {x_delta}")

        if self.baskets[self.basket_color].exists:
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

        x_sign = 1 if x_speed >= 0 else -1
        y_sign = 1 if y_speed >= 0 else -1
        rot_sign = 1 if rot_speed >= 0 else -1

        x_speed = min(abs(x_speed), self.max_speed) * x_sign
        y_speed = min(abs(y_speed), self.max_speed) * y_sign
        rot_speed = min(abs(rot_speed), self.max_speed) * rot_sign

        self.logger.log.info(f"--Orbiting-- MoveX {x_speed} MoveY {y_speed} rot {rot_speed}")

        self.robot.move(x_speed, y_speed, rot_speed)

    def ball_throw_state(self):
        """State for throwing the ball into the basket"""
        if time() > self.throw_end_time:  # end the throw after a specified amount of time
            self.thrower_substate = ThrowerState.EndThrow

        if self.thrower_substate == ThrowerState.StartThrow: # TODO - we're not checking if basket exists
            # all our data is from slightly away from the ball, so always adjusting the speed might been a bad idea. no idea if this works better
            self.thrower_speed = calculate_throw_speed(
                self.baskets[self.basket_color].distance)  # TODO - calibrate thrower
            self.thrower_substate = ThrowerState.MidThrow
            self.logger.log.info("--BallThrow-- Starting throw, basket distance:",
                  self.baskets[self.basket_color].distance, "speed:", self.thrower_speed)
            self.robot.move(0, self.throw_move_speed, 0, self.thrower_speed)
        elif self.thrower_substate == ThrowerState.MidThrow:
            rot_delta = self.middle_point - self.baskets[self.basket_color].x
            rot_speed = -1 * rot_delta * 0.003
            rot_sign = -1 if rot_speed >= 0 else 1
            rot_speed = min(abs(rot_speed), self.max_speed) * rot_sign
            self.robot.move(0, self.throw_move_speed,
                            rot_speed, self.thrower_speed)
        elif self.thrower_substate == ThrowerState.EndThrow:
            self.logger.log.info("--BallThrow-- Finishing throw.")
            self.back_to_search_state()
            self.thrower_substate = ThrowerState.Off
            self.thrower_speed = 0

    def parse_referee_cmd(self):
        """Function for parsing referee commands"""
        cmd = self.referee.get_cmd()
        if cmd == None:
            return

        if self.name in cmd["targets"]:
            robot_index = cmd["targets"].index(self.name)
            if robot_index > 1:
                return  # this should also never happen
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
                raise ValueError("Unknown basket colour:", basket_str)

            self.enemy_basket_color = Color(
                2) if self.basket_color == Color(3) else Color(3)
            self.back_to_search_state()
            self.logger.log.info("STARTING ROBOT, basket:", self.basket_color)

        elif cmd["signal"] == "stop":
            self.logger.log.info("STOPPING ROBOT")
            self.current_state = State.Stopped
        else:
            raise ValueError("Unknown signal:", cmd["signal"])

    def stop_state(self):
        """State for stopping."""
        self.robot.move(0, 0, 0, 0)

    def escape_from_basket_state(self):
        """State for escaping in case we get too close to the basket"""
        if self.escape_substate == EscapeState.StartEscape:
            self.escape_substate = EscapeState.Reverse
            self.escape_state_end = time() + 0.7  # dont reverse for too long
        elif self.escape_substate == EscapeState.Reverse:
            self.logger.log.info("--ESCAPE-- Reversing")
            if time() > self.escape_state_end:
                self.escape_substate = EscapeState.TurningFromBasket
                # rotate for 2 seconds, or until opposite basket is found
                self.escape_state_end = time() + 2
            else:
                self.robot.move(0, -self.max_speed * 0.75, 0)
        elif self.escape_substate == EscapeState.TurningFromBasket:
            self.logger.log.info("--ESCAPE-- Turning from basket")
            if (self.baskets[self.opposite_basket].exists):
                self.robot.stop()
                self.escape_substate = EscapeState.DrivingAway
                self.escape_state_end = time() + 1
            elif time() > self.escape_state_end:
                self.escape_substate = EscapeState.Off
                self.escape_state_end = 0
                self.back_to_search_state()  # go back to searching and hope for the best
            else:
                self.robot.move(0, 0, -self.max_speed)
        elif self.escape_substate == EscapeState.DrivingAway:
            self.logger.log.info("--ESCAPE-- Driving away")
            if time() > self.escape_state_end:
                self.escape_substate = EscapeState.Off
                self.escape_state_end = 0
                self.back_to_search_state()  # go back to searching and hope for the best
            else:
                self.robot.move(0, self.max_speed*0.75, 0)

    def main_loop(self):
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
        except Exception as e:
            self.logger.log.error("Error happened:", e)
        finally:
            self.robot.close()
            self.controller.stop()
            cv2.destroyAllWindows()
            self.processor.stop()
            self.referee.close()


if __name__ == "__main__":
    conf_debug = False
    conf_debug_data_collection = True
    conf_camera_deadzone = 5
    conf_max_speed = 1
    conf_throw_move_speed = 0.375
    conf_search_speed = 2
    conf_throw_time = 1.1037
    conf_min_distance = 340
    conf_max_ball_miss = 5
    conf_use_realsense = True
    conf_middle_offset = 0
    conf_basket_color = Color.BLUE
    conf_max_orbit_time = 15  # seconds
    conf_manual_thrower_speed = 1000  # default for remote control
    conf_controller_analog_deadzone = 400
    conf_referee_ip = "ws://192.168.3.69:8222"
    conf_name = "placeholder"
    conf_search_timeout = 3  # TODO - adjust
    conf_search_min_basket_dist = 1200  # TODO - adjust
    conf_avg_fps = 50 # for values that are tied to FPS in some way
    robot = Robot(conf_debug, conf_camera_deadzone, conf_max_speed, conf_search_speed, conf_throw_time,
                  conf_min_distance, conf_max_ball_miss, conf_use_realsense, conf_middle_offset, conf_basket_color, conf_max_orbit_time, conf_manual_thrower_speed, conf_controller_analog_deadzone, conf_debug_data_collection, conf_throw_move_speed, conf_referee_ip, conf_name, conf_search_timeout, conf_search_min_basket_dist, conf_avg_fps)
