import image_processor
import camera
import motion
import cv2
import time
from ds4_control import RobotDS4
from enum import Enum


class State(Enum):
    """State machine enums"""
    Searching = 1
    BallFound = 2
    Orbiting = 3
    BallThrow = 4
    Wait = 5
    RemoteControl = 6
    Debug = 99  # state for temporarily testing code


class Basket(Enum):
    """Basket enums"""
    BLUE = 1
    MAGENTA = 2


def main_loop():

    # -- CAMERA STUFF --
    debug = False  # Whether to show camera image or not
    # camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure=100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    middle_point = cam.rgb_width // 2
    # the middle area of the camera image
    camera_deadzone = 60
    # FPS counter
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0

    # -- ROBOT STUFF
    # start the robot
    robot = motion.OmniRobot()
    robot.open()
    max_speed = 0.75
    throw_wait = 5  # wait for 5 seconds
    max_distance = 500  # how far the ball has to be to prepare for throw
    scan_time = 1  # time to scan for balls when in search mode
    wait_end = 0
    # TODO - unhardcode this value eventually
    basket_color = Basket.BLUE
    # the state machine
    current_state = State.Searching
    next_state = None  # used for wait
    # initialize controller
    controller = RobotDS4(robot=robot)
    controller.start()
    search_end = time.time() + scan_time
    # probably not needed, just for debugging right now to cut down on log spam
    prev_ball_count = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            print("CURRENT STATE -", current_state)

            # -- REMOTE CONTROL STUFF --
            # stop button
            if controller.is_stopped():
                break

            # verify if robot is remote controlled, no autonomous stuff if robot is remote controlled
            if controller.is_remote_controlled():
                current_state = State.RemoteControl
            if current_state == State.RemoteControl:
                if not controller.is_remote_controlled():
                    current_state = State.Searching
                    search_end = time.time() + search_end
                else:
                    continue

            # -- AUTONOMOUS STUFF --

            ball_count = len(processedData.balls)
            if prev_ball_count != ball_count:
                # this is for debugging
                print("ball_count: {}".format(ball_count))
                prev_ball_count = ball_count

            # last element is always the largest and we want to chase the largest ball
            if ball_count > 0:
                ball = processedData.balls[-1]
            else:
                ball = None  # no ball

            # the state machine, very WIP and not really functional yet

            if current_state == State.Searching:
                if ball_count != 0:
                    current_state = State.BallFound
                else:
                    if time.time() < search_end:
                        robot.move(0, 0, max_speed, 0)
                    else:
                        current_state = State.Wait
                        wait_end = time.time() + scan_time
                        next_state = State.Searching

            if current_state == State.BallFound:
                # in case we lost the ball
                if ball_count == 0:
                    current_state = State.Searching
                    search_end = time.time() + search_end
                    continue

                if ball.x > (middle_point + camera_deadzone):
                    print("right")
                    robot.move(0, 0, -max_speed, 0)
                elif ball.x < (middle_point - camera_deadzone):
                    print("left")
                    robot.move(0, 0, max_speed, 0)
                else:
                    if ball.distance > max_distance:
                        # the greater the distance the closer the ball
                        print("ball close, distance:", ball.distance)
                        current_state = State.Orbiting
                    else:
                        robot.move(0, max_speed, 0, 0)

            if current_state == State.Orbiting:
                # in case we lost the ball
                if ball_count == 0:
                    current_state == State.Searching
                    search_end = time.time() + search_end
                    continue

                if basket_color == Basket.MAGENTA:
                    basket = processedData.basket_m
                elif basket_color == Basket.BLUE:
                    basket = processedData.basket_b

                if basket.exists:
                    current_state = State.Wait
                    next_state = State.BallThrow
                    wait_end = time.time() + throw_wait
                else:  # TODO - orbit here
                    if ball.x > (middle_point + camera_deadzone):
                        print("right")
                        robot.move(max_speed * 0.25, 0, -max_speed, 0)
                    elif ball.x < (middle_point - camera_deadzone):
                        print("left")
                        robot.move(max_speed * 0.25, 0, max_speed, 0)

            # TODO - actually get a thrower and implement thrower logic
            if current_state == State.BallThrow:
                print("straight")
                robot.move(0, max_speed, 0, 0)
                print("throw")
                current_state = State.Searching
                search_end = time.time() + search_end

            if current_state == State.Wait:
                if time.time() >= wait_end:
                    current_state = next_state
                    next_state = None
                    if current_state == State.Searching:
                        search_end = time.time() + search_end

            # USE THIS STATE ONLY FOR DEBUGGING STUFF, not intended for actual use
            if current_state == State.Debug:
                pass

            # -- BOILERPLATE CAMERA CODE, DO NOT TOUCH UNLESS REALLY NECESSARY --
            # FPS counter
            frame_cnt += 1
            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
            # Debug stuff, turn off when we don't need camera
            if debug:
                debug_frame = processedData.debug_frame
                cv2.imshow('debug', debug_frame)
                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break

    except KeyboardInterrupt:
        print("Closing....")
    finally:
        robot.close()
        controller.stop()
        cv2.destroyAllWindows()
        processor.stop()


main_loop()
