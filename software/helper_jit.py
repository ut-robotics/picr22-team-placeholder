# File for helper functions utilizing jit
from typing import List, Tuple
from numba import njit, prange
import numpy as np
from math import floor


@njit
def np_average_jit(array: List[float]) -> float:
    """Compute the weighted average along the specified axis.

    Args:
        array (List[float]): List to calculate the average from

    Returns:
        float: Weighted average
    """
    return np.average(array)


@njit
def np_zeros_jit(height: int, width: int) -> List[int]:
    """Return a new array of given shape, filled with zeros.

    Args:
        height (int): Array height
        width (int): Array width

    Returns:
        list: New array
    """
    return np.zeros(
        (height, width), dtype=np.uint8)


@njit(parallel=True, fastmath=True, cache=True)
def find_pixels_near_ball(
    image_fragments: np.ndarray,
    object_coords: Tuple[int, int, int, int],
    frag_size: Tuple[int, int],
    look_range: int
) -> Tuple[int, int, Tuple[int, int, int, int]]:
    """Returns how many black and white frames were found near the ball.

    Args:
        image_fragments (np.array): image fragments
        object_coords (tuple): coordinates for object, (x, y, w, h)
        frag_size (tuple): fragments size (y, x)
        look_range (int): how much to extend looking area compared to object size

    Returns:
        tuple: black_frame_count, white_frame_count, (x1, y1, x2, y2)
    """
    # Determine bounding box coordinates
    x1 = max(object_coords[0] - look_range, 0)
    y1 = max(object_coords[1] - look_range, 0)
    x2 = min(object_coords[0] + look_range, frag_size[1] - 1)
    y2 = min(
        floor(object_coords[1] + object_coords[2] + (look_range * 1.25)),
        450
    )

    # Count black and white frames within bounding box
    black_count = 0
    white_count = 0
    for x in prange(x1, x2):
        for y in prange(y1, y2):
            if image_fragments[y][x] == 6:
                black_count += 1
            elif image_fragments[y][x] == 5:
                white_count += 1

    return black_count, white_count, (x1, y1, x2, y2)

# TODO - Untested, test if this actually works on the robot


@njit
def get_average_distance(image_fragments: np.ndarray, depth: np.ndarray, color_id: int) -> float:
    # Convert the image_fragments and depth variables to NumPy arrays
    image_fragments = np.array(image_fragments)
    depth = np.array(depth)

    # Flatten the image_fragments and depth arrays to make them 1-dimensional
    image_fragments = image_fragments.flatten()
    depth = depth.flatten()

    # Use NumPy to find the indices of all pixels that have the specified color
    color_indices = np.where(image_fragments == color_id)

    # Use these indices to get the corresponding depth values for these pixels
    color_depths = depth[color_indices]

    # Use NumPy to calculate the average depth of the pixels with the specified color
    return np.mean(color_depths)
