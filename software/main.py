import image_processor
import camera
import motion
import cv2
from time import time
from ds4_control import RobotDS4
from enum import Enum
from Color import Color

class State(Enum):
    """State machine enums"""
    Searching = 1
    BallAlign = 2
    DriveToBall = 3
    Orbiting = 4
    BasketBallAlign = 5
    BallThrow = 6
    Wait = 7
    RemoteControl = 98
    Debug = 99  # state for temporarily testing code

class Robot:
    current_state = State.Searching
    next_state = None

    # FPS counter
    start = time()
    frame_cnt = 0

    robot = motion.OmniRobot()

    # probably not needed, just for debugging right now to cut down on log spam
    prev_ball_count = 0
    no_balls_frames = 0

    processed_data = 0
    ball_count = 0
    ball = 0

    def __init__(self,
                 debug: bool,
                 camera_deadzone: int,
                 max_speed: float,
                 search_speed: float,
                 throw_wait: int,
                 min_distance: int,
                 scan_wait_time: float,
                 scan_move_time: float,
                 max_ball_miss: int,
                 use_realsense: bool,
                 middle_offset: int,
                 basket_color: Color):
        self.debug = debug

        self.processor = image_processor.ImageProcessor(self.cam, debug=debug)
        self.processor.start()

        if use_realsense:
            # camera instance for realsense cameras
            self.cam = camera.RealsenseCamera(exposure=100)
        else:
            # camera instance for normal web cameras
            self.cam = camera.OpenCVCamera(id=2)

        self.middle_point = self.cam.rgb_width // 2 + middle_offset
        self.camera_deadzone = camera_deadzone

        self.robot.open()

        self.controller = RobotDS4(robot=self.robot)
        self.controller.start()

        self.max_speed = max_speed
        self.search_speed = search_speed
        self.throw_wait = throw_wait
        # how far the ball has to be to prepare for throw, approx 10 cm
        self.min_distance = min_distance

        # time to scan for balls when in search mode
        self.scan_wait_time = scan_wait_time
        self.scan_move_time = scan_move_time

        self.max_ball_miss = max_ball_miss

        self.basket_color = basket_color

        self.search_end = time() + scan_move_time
        self.wait_end = time() + scan_wait_time

    def back_to_search(self):
        """Returns to search state

            Args:
                scan_move_time (float): Time to move before stopping to scan
        """
        current_state = State.Searching
        search_end = time() + self.scan_move_time
        return current_state, search_end

    def get_image_data(self):
        # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
        self.processed_data = self.processor.process_frame(aligned_depth=True)

        # -- BOILERPLATE CAMERA CODE, DO NOT TOUCH UNLESS REALLY NECESSARY --
        # FPS counter
        self.frame_cnt += 1
        if self.frame_cnt % 30 == 0:
            self.frame_cnt = 0
            end = time()
            fps = 30 / (end - self.start)
            self.start = end
            print("FPS: {}, framecount: {}".format(fps, self.frame_cnt))
        # Debug stuff, turn off when we don't need camera
        if self.debug:
            debug_frame = self.processed_data.debug_frame
            cv2.imshow('debug', debug_frame)
            k = cv2.waitKey(1) & 0xff
            if k == ord('q'):
                raise KeyboardInterrupt

        # print("CURRENT STATE -", current_state)

        # -- REMOTE CONTROL STUFF --
        # stop button
        if self.controller.is_stopped():
            raise KeyboardInterrupt

        # verify if robot is remote controlled, no autonomous stuff if robot is remote controlled
        if self.controller.is_remote_controlled():
            self.current_state = State.RemoteControl
        if self.current_state == State.RemoteControl:
            if not self.controller.is_remote_controlled():
                self.current_state, self.search_end = self.back_to_search()
            else:
                raise KeyboardInterrupt

        # -- AUTONOMOUS STUFF --

        self.ball_count = len(self.processedData.balls)
        if self.prev_ball_count != self.ball_count:
            # this is for debugging
            print("ball_count: {}".format(self.ball_count))
            self.prev_ball_count = self.ball_count

        # last element is always the largest and we want to chase the largest ball
        if self.ball_count > 0:
            self.ball = self.processedData.balls[-1]
        else:
            self.ball = None  # no ball

    def searching_state(self):
        """State for searching for the ball"""
        if self.ball_count != 0:
            self.current_state = State.BallFound

        elif time() < self.search_end:
            print("--Searching-- Moving to look for ball")
            self.robot.move(0, 0, self.search_speed, 0)

        else:
            print("--Searching-- Entering wait to scan surroundings")
            self.current_state = State.Wait
            self.wait_end = time() + self.scan_wait_time
            self.next_state = State.Searching

    def wait_state(self):
        """State for waiting"""
        if self.next_state == State.Searching and self.ball_count != 0:
            print("--Wait-- Found ball.")
            self.current_state = State.BallAlign
            return
        if time() >= self.wait_end:
            if self.next_state is not None:
                self.current_state = self.next_state
            else:
                self.current_state = State.Searching
            self.next_state = None
            if self.current_state == State.Searching:
                self.search_end = time() + self.scan_move_time

    def ball_align_state(self):
        """State for aligning the robot with the ball's direction"""
        if self.no_balls_frames > self.max_ball_miss:  # lost the ball, TODO - increment the value
            self.current_state, self.search_end = self.back_to_search()
            return
        print("--BallAlign-- Ball found.")
        if self.ball.x > (self.middle_point + self.camera_deadzone):
            print("--BallAlign-- right")
            self.robot.move(0, 0, -self.max_speed, 0)
        elif self.ball.x < (self.middle_point - self.camera_deadzone):
            print("--BallAlign-- left")
            self.robot.move(0, 0, self.max_speed, 0)

        else:
            self.current_state = State.DriveToBall

    def orbiting_state(self):
        """State for orbiting around the ball and trying to find the basket."""
        if self.basket_color == Color.MAGENTA:
            basket = self.processed_data.basket_m
        elif self.basket_color == Color.BLUE:
            basket = self.processed_data.basket_b
            # TODO - adjust based on the ball
        if basket.exists:
            if basket.x < self.middle_point - 5:  # left
                self.robot.move(self.max_speed * 0.15, 0, self.max_speed * 0.5)
            elif basket.x > self.middle_point + 5:  # right
                self.robot.move(-self.max_speed * 0.15, 0, -self.max_speed * 0.5)
            else:
                self.current_state = State.BallThrow
        else:
            # move faster to try and find the basket
            self.robot.move(0.5, 0, 1.5)

    def backet_ball_align_state(self):
        """State for aligning the ball and the basket."""
        print("Unimplemented state BasketBallAlign.")

    def ball_throw_state(self):
        """State for throwing the ball into the basket"""
        if self.basket_color == Color.MAGENTA:
            basket = self.processed_data.basket_m
        elif self.basket_color == Color.BLUE:
            basket = self.processed_data.basket_b
        if basket.exists:  # TODO
            print("--BallThrow-- Throwing ball, basket distance:",
                  basket.distance)
        elif self.ball_count != 0:
            print("--BallThrow-- No basket, going back to orbiting.")
            current_state = State.Orbiting
        else:
            print("--BallThrow-- No basket or ball, going back to throwing.")
            current_state = State.Searching

    def remote_control_state(self):
        if not self.controller.is_remote_controlled():
            self.current_state, self.search_end = self.back_to_search()

    def main_loop(self):
        try:
            while True:
                self.get_image_data()

                if self.current_state == State.Searching:
                    self.searching_state()

                elif self.current_state == State.Wait:
                    self.wait_state()

                elif self.current_state == State.BallAlign:
                    self.ball_align_state()

                elif self.current_state == State.Orbiting:
                    self.orbiting_state()

                elif self.current_state == State.BasketBallAlign:
                    self.backet_ball_align_state()

                elif self.current_state == State.Orbiting:
                    self.orbiting_state()

                elif self.current_state == State.RemoteControl:
                    self.remote_control_state()

        except KeyboardInterrupt:
            print("Closing....")

        finally:
            self.robot.close()
            self.controller.stop()
            cv2.destroyAllWindows()
            self.processor.stop()

if __name__ == "__main__":
    conf_debug = True
    conf_camera_deadzone = 30
    conf_max_speed = 0.75
    conf_search_speed = 2
    conf_throw_wait = 5
    conf_min_distance = 340
    conf_scan_wait_time = 1
    conf_scan_move_time = 0.2
    conf_max_ball_miss = 5
    conf_use_realsense = True
    conf_middle_offset = 0
    conf_basket_color = Color.MAGENTA

    robot = Robot(conf_debug, conf_camera_deadzone, conf_max_speed, conf_search_speed, conf_throw_wait,
                  conf_min_distance, conf_scan_wait_time, conf_scan_move_time, )