import image_processor
import camera
import motion
import cv2
from time import time, sleep
from ds4_control import RobotDS4
from Color import Color
from states import State, ThrowerState
from helper import calculate_throw_speed


class Robot:
    """The main class for %placeholder%"""
    current_state = State.Searching
    thrower_substate = ThrowerState.Off
    thrower_speed = 0
    next_state = None

    # FPS counter
    start = time()
    frame_cnt = 0

    robot = motion.OmniRobot()

    # probably not needed, just for debugging right now to cut down on log spam
    prev_ball_count = 0
    no_balls_frames = 0
    orbit_start = 0
    throw_end_time = 0
    processed_data = 0
    ball_count = 0
    ball = None
    basket = None

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
                 throw_move_speed: float):
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
        self.max_orbit_time = max_orbit_time

        self.controller = RobotDS4(robot_data=self)
        self.controller.start()

        self.main_loop()

    def back_to_search_state(self):
        """Returns to search state
        """
        self.current_state = State.Searching

    def fps_counter(self):
        """Prints FPS"""
        self.frame_cnt += 1
        if self.frame_cnt % 30 == 0:
            self.frame_cnt = 0
            end = time()
            fps = 30 / (end - self.start)
            self.start = end
            #print("FPS: {}, framecount: {}".format(fps, self.frame_cnt))

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

        #print("CURRENT STATE -", self.current_state)

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
            #print("ball_count: {}".format(self.ball_count))
            self.prev_ball_count = self.ball_count

        # first element is always the closest ball
        if self.ball_count > 0:
            self.ball = self.processed_data.balls[0]
        elif self.no_balls_frames >= self.max_ball_miss:  # use old ball sometimes
            self.ball = None

        if self.basket_color == Color.MAGENTA:
            self.basket = self.processed_data.basket_m
        elif self.basket_color == Color.BLUE:
            self.basket = self.processed_data.basket_b

    def searching_state(self):
        """State for searching for the ball"""
        if self.ball_count != 0:
            self.robot.stop()
            self.current_state = State.DriveToBall
            return
        print("--Searching-- Moving to look for ball")
        self.robot.move(0, 0, self.search_speed, 0)

    def drive_to_ball_state(self):
        """State for driving to the ball."""
        if self.no_balls_frames >= self.max_ball_miss:  # lost the ball
            print(
                f"--DriveToBall-- Haven't seen ball for {self.max_ball_miss} frames, going back to search.")
            self.back_to_search_state()
            return
        print("--DriveToBall-- Driving to ball.")

        # TODO - might need to adjust these further
        if not self.min_distance + 35 > self.ball.distance > self.min_distance - 35:
            rot_delta = self.middle_point - self.ball.x
            y_delta = self.min_distance - self.ball.distance

            y_speed = -1 * y_delta * 0.0006
            rot_speed = -1 * rot_delta * 0.003

            y_sign = 1 if y_speed >= 0 else -1
            rot_sign = 1 if rot_speed >= 0 else -1

            y_speed = min(abs(y_delta), self.max_speed) * y_sign
            rot_speed = min(abs(rot_speed), self.max_speed) * rot_sign
            print(
                f"--DriveToBall-- ball distance {self.ball.distance}, y_speed {y_speed}, rot_speed {rot_speed}")
            self.robot.move(0, y_speed, rot_speed)
        else:
            self.robot.stop()
            self.current_state = State.Orbiting
            self.orbit_start = time()

    def orbiting_state(self):
        """State for orbiting around the ball and trying to find the basket."""
        if time() > self.orbit_start + self.max_orbit_time:
            print("--ORBIT-- Orbiting for too long, going back to search")
            self.robot.stop()
            self.back_to_search_state()
            return

        # TODO - put this into its own function, we reuse it a lot
        if self.no_balls_frames >= self.max_ball_miss:  # lost the ball
            print(
                f"--Orbiting-- Haven't seen ball for {self.max_ball_miss} frames, going back to search.")
            self.back_to_search_state()
            return

        if self.ball.distance > 3 * self.min_distance:
            self.state = State.Searching
            return

        # TODO - adjust these values to improve orbiting
        x_delta = self.middle_point - self.ball.x
        x_speed = -1 * x_delta * 0.0018

        y_delta = self.min_distance - self.ball.distance
        y_speed = -1 * y_delta * 0.001

        rot_speed = self.max_speed * 0.7

        print(f"--Orbiting-- Ball X {self.ball.x} Ball X delta {x_delta}")

        if self.basket.exists:
            basket_delta = self.basket.x - self.middle_point

            print(f"--Orbiting-- Basket delta {basket_delta}")

            rot_speed = -1 * basket_delta * 0.009
            # TODO - a bit too sensitive at long distances
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

        print(f"--Orbiting-- MoveX {x_speed} MoveY {y_speed} rot {rot_speed}")

        self.robot.move(x_speed, y_speed, rot_speed)

    def ball_throw_state(self):
        """State for throwing the ball into the basket"""
        if time() > self.throw_end_time:  # end the throw after a specified amount of time
            self.thrower_substate = ThrowerState.EndThrow

        if self.thrower_substate == ThrowerState.StartThrow:
            # all our data is from slightly away from the ball, so always adjusting the speed might been a bad idea. no idea if this works better
            self.thrower_speed = calculate_throw_speed(
                self.basket.distance)  # TODO - calibrate thrower
            self.thrower_substate = ThrowerState.MidThrow
            print("--BallThrow-- Starting throw, basket distance:",
                  self.basket.distance, "speed:", self.thrower_speed)
            self.robot.move(0, self.throw_move_speed, 0, self.thrower_speed)
        elif self.thrower_substate == ThrowerState.MidThrow:
            print("--BallThrow-- Throwing ball, basket distance:",
                  self.basket.distance, "speed:", self.thrower_speed)
            self.robot.move(0, self.throw_move_speed, 0, self.thrower_speed)
        elif self.thrower_substate == ThrowerState.EndThrow:
            print("--BallThrow-- Finishing throw.")
            self.current_state = State.Searching
            self.thrower_substate = ThrowerState.Off
            self.thrower_speed = 0

    def main_loop(self):
        try:
            while True:
                self.get_image_data()

                if self.current_state == State.RemoteControl:
                    continue

                elif self.current_state == State.Searching:
                    self.searching_state()

                elif self.current_state == State.Orbiting:
                    self.orbiting_state()

                elif self.current_state == State.DriveToBall:
                    self.drive_to_ball_state()

                elif self.current_state == State.BallThrow:
                    self.ball_throw_state()

        except KeyboardInterrupt:
            print("Closing....")

        finally:
            self.robot.close()
            self.controller.stop()
            cv2.destroyAllWindows()
            self.processor.stop()


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

    robot = Robot(conf_debug, conf_camera_deadzone, conf_max_speed, conf_search_speed, conf_throw_time,
                  conf_min_distance, conf_max_ball_miss, conf_use_realsense, conf_middle_offset, conf_basket_color, conf_max_orbit_time, conf_manual_thrower_speed, conf_controller_analog_deadzone, conf_debug_data_collection, conf_throw_move_speed)
