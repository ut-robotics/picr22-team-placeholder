from pathlib import Path
import _pickle as pickle
import numpy as np
import cv2
from modules.camera import RealsenseCamera
from modules.image_processor import ImageProcessor
from modules.helper import get_colors_pkl_path, load_config
from modules.Color import *
from modules.logger import Logger
"""
   _____ _____ _____ _____ 
  |     |     |     |     |
  | %   | p   | l   | a   |
  |_____|_____|_____|_____|
  |     |     |     |     |
  | h   | o   | l   | d   |
  |_____|_____|_____|_____|
  |     |     |     |     |
  | e   | r   | c   | e   |
  |_____|_____|_____|_____|
  '); DROP TABLE BOT

"""


def nothing(x):
    pass


config = load_config()
# how many times we can possibly undo
MAX_HISTORY = config["camera"]["undo_count"]
logger = Logger(config["logging"]["log_level"], name="%ColourConfig%")
cv2.namedWindow('image')
cv2.namedWindow('debug')
cv2.namedWindow('mask')
cv2.moveWindow('mask', 400, 0)

pkl_path = get_colors_pkl_path()

try:
    with open(pkl_path, 'rb') as fh:
        colors_lookup = pickle.load(fh)
except:
    # make the folder in case it does not exist
    Path(pkl_path).parents[0].mkdir(parents=True, exist_ok=True)
    colors_lookup = np.zeros(0x1000000, dtype=np.uint8)
old_lookups = list()
cap = RealsenseCamera(exposure=100)

processor = ImageProcessor(
    cap, logger=logger, debug=True, config=config, colors_lookup=colors_lookup)

cv2.createTrackbar('brush_size', 'image', 3, 10, nothing)
cv2.createTrackbar('noise', 'image', 1, 5, nothing)

mouse_x = 0
mouse_y = 0
brush_size = 1
noise = 1
p = 0

keyDict = {
    ord("g"): Color.GREEN,
    ord("m"): Color.MAGENTA,
    ord("b"): Color.BLUE,
    ord("f"): Color.ORANGE,
    ord("w"): Color.WHITE,
    ord("d"): Color.BLACK,
    ord("o"): Color.OTHER,
}


def change_color(noise, brush_size, mouse_x, mouse_y):
    ob = rgb[
        max(0, mouse_y-brush_size):min(cap.rgb_height, mouse_y+brush_size+1),
        max(0, mouse_x-brush_size):min(cap.rgb_width, mouse_x+brush_size+1), :].reshape((-1, 3)).astype('int32')
    noises = range(-noise, noise+1)
    add_undo()
    for r in noises:
        for g in noises:
            for b in noises:
                colors_lookup[((ob[:, 0]+r) + (ob[:, 1]+g) * 0x100 +
                               (ob[:, 2]+b) * 0x10000).clip(0, 0xffffff)] = p
    processor.set_segmentation_table(processor.colors_lookup)

# mouse callback function


def choose_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse_x = x
        mouse_y = y
        brush_size = cv2.getTrackbarPos('brush_size', 'image')
        noise = cv2.getTrackbarPos('noise', 'image')
        change_color(noise, brush_size, mouse_x, mouse_y)


def add_undo():
    old_lookups.append(colors_lookup.copy())
    if len(old_lookups) > MAX_HISTORY:
        old_lookups.pop(0)  # remove oldest item


cv2.setMouseCallback('debug', choose_color)
cv2.setMouseCallback('mask', choose_color)

logger.log.info("Quit: 'q', Save 's', Erase selected color 'e', Undo 'u'")
logger.log.info(
    "Balls 'g', Magenta basket='m', Blue basket='b', Field='f', White='w', Black='d', Other='o'")

cap.open()

while (True):
    processed_data = processor.process_frame()

    rgb = processed_data.color_frame

    fragmented = colors_lookup[rgb[:, :, 0] +
                               rgb[:, :, 1] * 0x100 + rgb[:, :, 2] * 0x10000]
    frame = np.zeros((cap.rgb_height, cap.rgb_width, 3), dtype=np.uint8)

    for color in Color:
        frame[fragmented == int(color)] = color.color

    cv2.imshow('mask', frame)

    debug_frame = processed_data.debug_frame
    cv2.imshow('debug', debug_frame)

    k = cv2.waitKey(1) & 0xff

    if k == ord('q'):
        break
    elif k == ord('u'):
        if len(old_lookups) <= 0:
            logger.log.warning("No more undos!")
        else:
            logger.log.info("Undo'd.")
            np.copyto(colors_lookup, old_lookups.pop())
            processor.set_segmentation_table(processor.colors_lookup)
    elif k in keyDict:
        col = keyDict[k]
        logger.log.info(f"Switched to: {col}")
        p = int(col)
    elif k == ord('s'):
        with open(pkl_path, 'wb') as fh:
            pickle.dump(colors_lookup, fh, -1)
        logger.log.info('Saved data to file.')
    elif k == ord('e'):
        logger.log.info('Erased color.')
        add_undo()
        colors_lookup[colors_lookup == p] = 0
        processor.set_segmentation_table(processor.colors_lookup)

# When everything done, release the capture
logger.log.info("Closing...")
cap.close()

cv2.destroyAllWindows()
