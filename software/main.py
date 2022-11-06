import image_processor
import camera
import motion
import cv2
import time
from ds4_control import RobotDS4
from enum import Enum
from Color import Color


class State(Enum):
    """State machine enums"""
    Searching = 1
    BallFound = 2
    Orbiting = 3
    BallThrow = 4
    Wait = 5
    BallAlign = 6
    DriveToBall = 7
    BasketBallAlign = 8
    RemoteControl = 98
    Debug = 99  # state for temporarily testing code


def back_to_search(scan_move_time):
    """Returns to search state

        Args:
            scan_move_time (int): Time to move before stopping to scan
    """
    current_state = State.Searching
    search_end = time.time() + scan_move_time
    return current_state, search_end


def main_loop():

    # -- CAMERA STUFF --
    debug = True  # Whether to show camera image or not
    # camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure=100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    middle_point = cam.rgb_width // 2
    # the middle area of the camera image
    camera_deadzone = 30
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
    search_speed = 2
    throw_wait = 5  # wait for 5 seconds
    min_distance = 340  # how far the ball has to be to prepare for throw, approx 10 cm
    scan_wait_time = 1  # time to scan for balls when in search mode
    scan_move_time = 0.2
    wait_end = 0

    max_ball_miss = 5
    # TODO - unhardcode this value eventually
    basket_color = Color.MAGENTA
    # the state machine
    current_state = State.Searching
    next_state = None  # used for wait
    # initialize controller
    controller = RobotDS4(robot=robot)
    controller.start()
    search_end = time.time() + scan_move_time
    # probably not needed, just for debugging right now to cut down on log spam
    prev_ball_count = 0
    no_balls_frames = 0
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=True)

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

            #print("CURRENT STATE -", current_state)

            # -- REMOTE CONTROL STUFF --
            # stop button
            if controller.is_stopped():
                break

            # verify if robot is remote controlled, no autonomous stuff if robot is remote controlled
            if controller.is_remote_controlled():
                current_state = State.RemoteControl
            if current_state == State.RemoteControl:
                if not controller.is_remote_controlled():
                    current_state, search_end = back_to_search(scan_move_time)
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
                        print("--Searching-- Moving to look for ball")
                        robot.move(0, 0, search_speed, 0)
                    else:
                        print("--Searching-- Entering wait to scan surroundings")
                        current_state = State.Wait
                        wait_end = time.time() + scan_wait_time
                        next_state = State.Searching

            if current_state == State.Wait:
                if next_state == State.Searching and ball_count != 0:
                    print("--Wait-- Found ball.")
                    current_state = State.BallFound
                    next_state = None
                    continue
                if time.time() >= wait_end:
                    if next_state != None:
                        current_state = next_state
                    else:
                        current_state = State.Searching
                    next_state = None
                    if current_state == State.Searching:
                        search_end = time.time() + scan_move_time
                else:
                    # print("--Wait-- Waiting for ",
                    #      wait_end - time.time(), "seconds.")
                    pass
            if current_state == State.BallFound:
                if no_balls_frames > max_ball_miss:  # lost the ball
                    current_state, search_end = back_to_search(scan_move_time)
                    continue
                print("--BallFound-- Ball found.")
                if ball.x > (middle_point + camera_deadzone):
                    print("--BallFound-- right")
                    robot.move(0, 0, -max_speed, 0)
                elif ball.x < (middle_point - camera_deadzone):
                    print("--BallFound-- left")
                    robot.move(0, 0, max_speed, 0)

                else:
                    if ball.distance < min_distance:
                        # the greater the distance the closer the ball
                        print("--BallFound-- ball close, distance:", ball.distance)
                        current_state = State.Orbiting
                    else:
                        print("--BallFound-- ball far, distance:", ball.distance)
                        robot.move(0, max_speed, 0)

            if current_state == State.Orbiting:
                """if ball_count == 0:
                    current_state, search_end = back_to_search(scan_move_time)
                    continue

                print("--ORBITING-- starting orbiting")"""
                if basket_color == Color.MAGENTA:
                    basket = processedData.basket_m
                elif basket_color == Color.BLUE:
                    basket = processedData.basket_b

                # TODO - adjust based on the ball
                if basket.exists:
                    if basket.x < middle_point - 5: # left
                        robot.move(max_speed*0.15, 0, max_speed*0.5)
                    elif basket.x > middle_point + 5: # right
                        robot.move(-max_speed*0.15, 0, -max_speed*0.5)
                    else:
                        current_state = State.BallThrow
                    
                else:
                    # move faster to try and find the basket
                    robot.move(0.5, 0, 1.5)

            # TODO - actually get a thrower
            if current_state == State.BallThrow:
                if basket_color == Color.MAGENTA:
                    basket = processedData.basket_m
                elif basket_color == Color.BLUE:
                    basket = processedData.basket_b
                if basket.exists:  # TODO
                    print("--BallThrow-- Throwing ball, basket distance:",
                          basket.distance)
                elif ball_count != 0:
                    print("--BallThrow-- No basket, going back to orbiting.")
                    current_state = State.Orbiting
                else:
                    print("--BallThrow-- No basket or ball, going back to throwing.")
                    current_state = State.Searching

            # USE THIS STATE ONLY FOR DEBUGGING STUFF, not intended for actual use
            if current_state == State.Debug:
                pass

    except KeyboardInterrupt:
        print("Closing....")
    finally:
        robot.close()
        controller.stop()
        cv2.destroyAllWindows()
        processor.stop()


main_loop()
