import camera
import segment
import _pickle as pickle
import numpy as np
import cv2
import Color as c
from helper_jit import find_black_near_ball, np_zeros_jit, np_average_jit
from helper import get_colors_pkl_path


class Object():
    def __init__(self, x=-1, y=-1, size=-1, distance=-1, exists=False):
        self.x = x
        self.y = y
        self.size = size
        self.distance = distance
        self.exists = exists

    def __str__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)

    def __repr__(self) -> str:
        return "[Object: x={}; y={}; size={}; distance={}; exists={}]".format(self.x, self.y, self.size, self.distance, self.exists)


# results object of image processing. contains coordinates of objects and frame data used for these results
class ProcessedResults():

    def __init__(self,
                 balls=[],
                 basket_b=Object(exists=False),
                 basket_m=Object(exists=False),
                 color_frame=[],
                 depth_frame=[],
                 fragmented=[],
                 debug_frame=[]) -> None:

        self.balls = balls
        self.basket_b = basket_b
        self.basket_m = basket_m
        self.color_frame = color_frame
        self.depth_frame = depth_frame
        self.fragmented = fragmented

        # can be used to illustrate things in a separate frame buffer
        self.debug_frame = debug_frame


# Main processor class. processes segmented information
class ImageProcessor():
    def __init__(self, camera, logger, color_config=get_colors_pkl_path(), debug=False):
        self.camera = camera

        self.color_config = color_config
        with open(self.color_config, 'rb') as conf:
            self.colors_lookup = pickle.load(conf)
            self.set_segmentation_table(self.colors_lookup)
        self.fragmented = np_zeros_jit(
            self.camera.rgb_height, self.camera.rgb_width)

        self.t_balls = np_zeros_jit(
            self.camera.rgb_height, self.camera.rgb_width)
        self.t_basket_b = np_zeros_jit(
            self.camera.rgb_height, self.camera.rgb_width)
        self.t_basket_m = np_zeros_jit(
            self.camera.rgb_height, self.camera.rgb_width)
        self.logger = logger
        self.debug = debug
        self.debug_frame = np_zeros_jit(
            self.camera.rgb_height, self.camera.rgb_width)

    def set_segmentation_table(self, table):
        segment.set_table(table)

    def start(self):
        self.camera.open()

    def stop(self):
        self.camera.close()

    def analyze_balls(self, t_balls, depth, fragments, basket) -> list:
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        t_balls = cv2.dilate(t_balls, kernel)
        t_balls = cv2.erode(t_balls, kernel)
        contours, hierarchy = cv2.findContours(
            t_balls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        balls = []
        frag_x, frag_y = np.shape(fragments)
        for contour in contours:

            # ball filtering logic goes here. Example includes filtering by size and an example how to get pixels from
            # the bottom center of the frame to the ball

            size = cv2.contourArea(contour)

            if size < 15:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            black_count = find_black_near_ball(
                fragments, (x, y, w, h), (frag_x, frag_y), 50)
            if black_count > 180:  # skip the ball if its on the black part of the arena. this is not as good as line detection but good enough for now
                continue

            ys = np.array(
                np.arange(y + h, self.camera.rgb_height), dtype=np.uint16)
            xs = np.array(np.linspace(
                x + w/2, self.camera.rgb_width / 2, num=len(ys)), dtype=np.uint16)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            if depth is None:
                obj_dst = obj_y
            else:
                try:
                    obj_dst = np_average_jit(
                        depth[obj_y-2:obj_y+2, obj_x-2:obj_x+2])
                except (ZeroDivisionError):
                    self.logger.log.error(
                        "Ball attempted to divide by zero when averaging.")
                    continue

            # don't add if ball is further than the basket or too close to it
            if basket != None:
                if 160 < basket.x < 600:
                    if basket.distance - 500 <= obj_dst:
                        continue

            if self.debug:
                self.debug_frame[ys, xs] = [0, 0, 0]
                cv2.circle(self.debug_frame, (obj_x, obj_y),
                           10, (0, 255, 0), 2)

            balls.append(Object(x=obj_x, y=obj_y, size=size,
                         distance=obj_dst, exists=True))

        balls.sort(key=lambda x: x.distance)

        return balls

    def analyze_baskets(self, t_basket, depth,  debug_color=(0, 255, 255)) -> list:
        kernel = np.ones((3, 3), np.uint8)
        t_basket = cv2.morphologyEx(t_basket, cv2.MORPH_CLOSE, kernel)
        contours, hierarchy = cv2.findContours(
            t_basket, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        baskets = []

        for contour in contours:

            # basket filtering logic goes here. Example includes size filtering of the basket

            size = cv2.contourArea(contour)

            if size < 100:
                continue

            x, y, w, h = cv2.boundingRect(contour)

            obj_x = int(x + (w/2))
            obj_y = int(y + (h/2))
            if depth is None:
                obj_dst = obj_y
            else:
                try:
                    obj_dst = np_average_jit(
                        depth[obj_y-5:obj_y+5, obj_x-5:obj_x+5])
                except (ZeroDivisionError):
                    self.logger.log.error(
                        "Basket attempted to divide by zero when averaging.")
                    continue

            baskets.append(Object(x=obj_x, y=obj_y, size=size,
                           distance=obj_dst, exists=True))

        baskets.sort(key=lambda x: x.size)

        basket = next(iter(baskets), Object(exists=False))

        if self.debug:
            if basket.exists:
                cv2.circle(self.debug_frame, (basket.x, basket.y),
                           20, debug_color, -1)

        return basket

    def get_frame_data(self, aligned_depth=False):
        if self.camera.has_depth_capability():
            return self.camera.get_frames(aligned=aligned_depth)
        else:
            return self.camera.get_color_frame(), np.zeros((self.camera.rgb_height, self.camera.rgb_width), dtype=np.uint8)

    def process_frame(self, aligned_depth=False) -> ProcessedResults:
        color_frame, depth_frame = self.get_frame_data(
            aligned_depth=aligned_depth)

        segment.segment(color_frame, self.fragmented,
                        self.t_balls, self.t_basket_m, self.t_basket_b)

        if self.debug:
            self.debug_frame = np.copy(color_frame)
        basket_b = self.analyze_baskets(
            self.t_basket_b, depth_frame, debug_color=c.Color.BLUE.color.tolist())
        basket_m = self.analyze_baskets(
            self.t_basket_m, depth_frame, debug_color=c.Color.MAGENTA.color.tolist())
        if basket_b.exists and basket_m.exists:
            basket_to_check = basket_b if basket_b.distance > basket_m.distance else basket_m
        elif True in [basket_b.exists, basket_m.exists]:
            basket_to_check = basket_b if basket_b.exists else basket_m
        else:
            basket_to_check = None

        balls = self.analyze_balls(
            self.t_balls, depth_frame, self.fragmented, basket_to_check)

        return ProcessedResults(balls=balls,
                                basket_b=basket_b,
                                basket_m=basket_m,
                                color_frame=color_frame,
                                depth_frame=depth_frame,
                                fragmented=self.fragmented,
                                debug_frame=self.debug_frame)
