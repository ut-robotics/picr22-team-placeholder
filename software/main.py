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
    DriveToBall = 2
    Orbiting = 3
    BallThrow = 4
    Wait = 5
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
    orbit_start = 0
    wait_end = 0
    processed_data = 0
    ball_count = 0
    ball = None
    basket = None

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
                 basket_color: Color,
                 max_orbit_time: int):
        self.debug = debug

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
        self.max_orbit_time = max_orbit_time

        self.search_end = time() + scan_move_time
        self.main_loop()

    def back_to_search_state(self):
        """Returns to search state
        """
        self.current_state = State.Searching
        self.search_end = time() + self.scan_move_time

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

        print("CURRENT STATE -", self.current_state)

        # -- REMOTE CONTROL STUFF --
        # stop button
        if self.controller.is_stopped():
            raise KeyboardInterrupt

        # verify if robot is remote controlled, no autonomous stuff if robot is remote controlled
        if self.controller.is_remote_controlled():
            self.current_state = State.RemoteControl
        if self.current_state == State.RemoteControl:
            if not self.controller.is_remote_controlled():
                self.back_to_search_state()
            else:
                return

        # -- AUTONOMOUS STUFF --
        if self.ball_count == 0:
            self.no_balls_frames += 1
        else:
            self.no_balls_frames = 0

        self.ball_count = len(self.processed_data.balls)
        if self.prev_ball_count != self.ball_count:
            # this is for debugging
            print("ball_count: {}".format(self.ball_count))
            self.prev_ball_count = self.ball_count

        # first element is always the closest ball
        if self.ball_count > 0:
            self.ball = self.processed_data.balls[0]
        else:
            self.ball = None

        if self.basket_color == Color.MAGENTA:
            self.basket = self.processed_data.basket_m
        elif self.basket_color == Color.BLUE:
            self.basket = self.processed_data.basket_b

    def searching_state(self):
        """State for searching for the ball"""
        if self.ball_count != 0:
            self.current_state = State.DriveToBall

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
            self.current_state = State.DriveToBall
            return
        if time() >= self.wait_end:
            if self.next_state is not None:
                self.current_state = self.next_state
            else:
                self.current_state = State.Searching
            self.next_state = None
            if self.current_state == State.Searching:
                self.search_end = time() + self.scan_move_time

    def drive_to_ball_state(self):
        """State for driving to the ball."""
        if self.no_balls_frames > self.max_ball_miss:  # lost the ball
            print(
                f"--DriveToBall-- Haven't seen ball for {self.max_ball_miss} frames, going back to search.")
            self.back_to_search_state()
            return
        if self.ball_count == 0:  # don't do anything when we cant see a ball
            self.robot.stop()
            return
        print("--DriveToBall-- Driving to ball.")
        
        rot_speed = 0
        
        if abs(self.ball.x - self.middle_point) > self.camera_deadzone:
            delta = self.middle_point - self.ball.x
            rot_speed = delta * 0.007
        
        if self.ball.distance < self.min_distance - 100:
            print("--DriveToBall-- ball too close, distance:", self.ball.distance)
            self.robot.move(0, -self.max_speed * 0.25, rot_speed)
        elif self.ball.distance > self.min_distance * 2:
            print("--DriveToBall-- ball far, distance:", self.ball.distance)
            self.robot.move(0, self.max_speed, rot_speed)
        elif self.ball.distance > self.min_distance * 1.25:
            self.robot.move(0, self.max_speed*0.5, rot_speed)
        else:
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
        if self.no_balls_frames > self.max_ball_miss:  # lost the ball
            print(
                f"--Orbiting-- Haven't seen ball for {self.max_ball_miss} frames, going back to search.")
            self.back_to_search_state()
            return
        if self.ball_count == 0:  # don't do anything when we cant see a ball
            self.robot.stop()
            return
        
        if self.ball.distance > 4 * self.min_distance:
            self.state = State.Searching
            return
        
        x_delta = self.middle_point - self.ball.x
        x_speed = -1 * x_delta * 0.001
        
        y_delta = self.min_distance - self.ball.distance
        y_speed = -1 * y_delta * 0.005
        
        rot_speed = self.max_speed * 0.7
        
        print(f"--Orbiting-- Ball X {self.ball.x} Ball Y {self.ball.distance}")
        
        if self.basket.exists:
            basket_delta = self.basket.x - self.middle_point
            
            print(f"--Orbiting-- Basket X {self.basket.x}")
            
            rot_speed = -1 * basket_delta * 0.004
            
            if abs(x_delta) <= 5 and abs(basket_delta) <= 5:
                self.current_state = State.BallThrow
                return
        
        x_speed = min(x_speed, self.max_speed)
        y_speed = min(y_speed, self.max_speed)
        rot_speed = min(rot_speed, self.max_speed)
        
        print(f"--Orbiting-- MoveX {x_speed} MoveY {y_speed} rot {rot_speed}")
        
        self.robot.move(x_speed, y_speed, rot_speed)

    def ball_throw_state(self):
        """State for throwing the ball into the basket"""
        if self.basket.exists:  # TODO
            print("--BallThrow-- Throwing ball, basket distance:",
                  self.basket.distance)
        elif self.ball_count != 0:
            print("--BallThrow-- No basket, going back to orbiting.")
            self.current_state = State.Orbiting
            self.orbit_start = time()
        else:
            print("--BallThrow-- No basket or ball, going back to throwing.")
            self.current_state = State.Searching

    def remote_control_state(self):
        if not self.controller.is_remote_controlled():
            self.back_to_search_state()

    def main_loop(self):
        try:
            while True:
                self.get_image_data()

                if self.current_state == State.Searching:
                    self.searching_state()

                elif self.current_state == State.Wait:
                    self.wait_state()

                elif self.current_state == State.Orbiting:
                    self.orbiting_state()

                elif self.current_state == State.RemoteControl:
                    self.remote_control_state()

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
    conf_debug = True
    conf_camera_deadzone = 5
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
    conf_max_orbit_time = 15  # seconds

    robot = Robot(conf_debug, conf_camera_deadzone, conf_max_speed, conf_search_speed, conf_throw_wait,
                  conf_min_distance, conf_scan_wait_time, conf_scan_move_time, conf_max_ball_miss, conf_use_realsense, conf_middle_offset, conf_basket_color, conf_max_orbit_time)
