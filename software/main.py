import image_processor
import camera
import motion # TODO - hook up with motion after we confirm it works
import cv2
import time

def main_loop():
    debug = True
    #camera instance for normal web cameras
    #cam = camera.OpenCVCamera(id = 2)
    # camera instance for realsense cameras
    cam = camera.RealsenseCamera(exposure = 100)
    
    processor = image_processor.ImageProcessor(cam, debug=debug)

    processor.start()
    middle_point = cam.rgb_width // 2
    deadzone = 60
    robot = motion.OmniRobot()
    robot.open()
    start = time.time()
    fps = 0
    frame = 0
    frame_cnt = 0
    max_speed = 1
    try:
        while True:
            # has argument aligned_depth that enables depth frame to color frame alignment. Costs performance
            processedData = processor.process_frame(aligned_depth=False)

            # This is where you add the driving behaviour of your robot. It should be able to filter out
            # objects of interest and calculate the required motion for reaching the objects

            frame_cnt +=1

            frame += 1
            if frame % 30 == 0:
                frame = 0
                end = time.time()
                fps = 30 / (end - start)
                start = end
                print("FPS: {}, framecount: {}".format(fps, frame_cnt))
                print("ball_count: {}".format(len(processedData.balls)))
                for ball in processedData.balls:
                    if ball.x > (middle_point + deadzone):
                        print("right")
                        robot.move(0, 0, -max_speed, 0)
                    elif ball.x < (middle_point - deadzone):
                        print("left")
                        robot.move(0, 0, max_speed, 0)
                    else:
                        print("straight")
                        robot.move(0, max_speed, 0, 0)
                    print(ball.x)

            if debug:
                debug_frame = processedData.debug_frame

                cv2.imshow('debug', debug_frame)

                k = cv2.waitKey(1) & 0xff
                if k == ord('q'):
                    break
    except KeyboardInterrupt:
        print("closing....")
        robot.stop()
        robot.close()
    finally:
        cv2.destroyAllWindows()
        processor.stop()

main_loop()
