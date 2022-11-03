import image_processor
import camera
import motion
import cv2
import time
from ds4_control import RobotDS4
from enum import Enum


class State(Enum):
    Searching = 1
    BallFound = 2
    Orbiting = 3
    BallThrow = 4
    Wait = 5
    RemoteControl = 6
    Debug = 99 # state for temporarily testing code


class Basket(Enum):
    BLUE = 1
    MAGENTA = 2


def main_loop():
    debug = False # whether to show camera image or not
    # camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure=100)
    processor = image_processor.ImageProcessor(cam, debug=debug)
    processor.start()
    middle_point = cam.rgb_width // 2
    # the middle area of the camera image
    camera_deadzone = 60
    # start the robot
    robot = motion.OmniRobot()
    robot.open()
    
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    max_speed = 1
    # initialize controller
    controller = RobotDS4(robot=robot)
    controller.start()
    if controller == None:
        print("Failed to initialize controller!")
    # the funny state machine
    current_state = State.Searching
    # TODO - unhardcode this value eventually
    basket_color = Basket.BLUE
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)
            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

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
            
            print("CURRENT STATE -", current_state)
            if current_state == State.RemoteControl:
                continue
            # no autonomous mode if robot is remote controlled
            if controller != None:
                if controller.is_stopped():
                    break
                elif controller.is_remote_controlled():
                    current_state == State.RemoteControl
            # autonomous code here
            ball_count = len(processedData.balls)
            print("ball_count: {}".format(ball_count))
            
            # the state machine, very WIP
            if current_state == State.Searching:
                if ball_count != 0:
                    current_state = State.BallFound
                robot.move(0, 0, max_speed, 0)

            if current_state == State.BallFound:
                if ball_count == 0:
                    current_state = State.Searching
                for ball in processedData.balls:
                    if ball.x > (middle_point + camera_deadzone):
                        print("right")
                        robot.move(0, 0, -max_speed, 0)
                    elif ball.x < (middle_point - camera_deadzone):
                        print("left")
                        robot.move(0, 0, max_speed, 0)
                    else:
                        #current_state = State.Orbiting
                        current_state = State.BallThrow
                    print(ball.x)

            if current_state == State.Orbiting:
                if ball_count == 0:
                    current_state == State.Searching
                if basket_color == Basket.MAGENTA:
                    basket = processedData.basket_m
                elif basket_color == Basket.BLUE:
                    basket = processedData.basket_b
                if basket.exists:
                    current_state = State.BallThrow
                else:
                    pass  # TODO - orbit here
                    robot.move(max_speed * 0.25, 0, max_speed, 0) # TODO - this doesnt work at all, figure out how to actually write orbit code

            if current_state == State.BallThrow:
                print("straight")
                robot.move(0, max_speed, 0, 0)
                # TODO - actually get a thrower and implement thrower logic
                print("throw")
                current_state = State.Searching

            # USE THIS STATE ONLY FOR DEBUGGING STUFF, not intended for actual use
            if current_state == State.Debug:
                if basket_color == Basket.MAGENTA:
                    basket = processedData.basket_m
                elif basket_color == Basket.BLUE:
                    basket = processedData.basket_b
                if basket.exists:
                    print(f'X: {basket.x}, Y: {basket.y}, distance: {basket.distance}, size: {basket.size}')
                else:
                    print("no basket :(((")

    except KeyboardInterrupt:
        print("closing....")
        robot.stop()
        robot.close()
        controller.stop
    finally:
        cv2.destroyAllWindows()
        processor.stop()


main_loop()
