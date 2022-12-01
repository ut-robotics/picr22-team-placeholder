# File for helper functions utilizing jit
from numba import njit, prange
import numpy as np


@njit
def np_average_jit(array):
    """Compute the weighted average along the specified axis.

    Args:
        array (list): List to calculate the average from

    Returns:
        float: Weighted average
    """
    return np.average(array)

@njit
def np_zeros_jit(height, width):
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
def find_black_near_ball(fragments, object_coords, frag_size, look_range):
    """Returns how many black frames were found near the ball

    Args:
        fragments (np.array): fragments
        object_coords (tuple): coordinates for object, (x, y, w, h)
        frag_size (tuple): fragments size (y, x)
        look_range (int): how much to extend looking area compared to object size

    Returns:
        int: black_frame_count
    """
    x1 = object_coords[0] - look_range
    y1 = object_coords[1] - look_range
    x2 = object_coords[0] + object_coords[2] + look_range
    y2 = object_coords[1] + object_coords[3] + look_range
    # make sure it doesn't go out of bounds
    if x1 < 0:
        x1 = 0
    if y1 < 0:
        y1 = 0
    if x2 > frag_size[1]:
        x2 = frag_size[1] - 1
    if y2 > 450:
        y2 = 450

    black_count = 0
    for x in prange(x1, x2):
        for y in prange(y1, y2):
            if fragments[y][x] == 6:
                black_count += 1
    return black_count
