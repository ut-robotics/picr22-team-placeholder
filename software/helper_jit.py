# File for helper functions utilizing jit
from typing import List, Tuple
from numba import njit, prange
import numpy as np

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
def find_black_near_ball(
    image_fragments: np.array,
    object_coords: Tuple[int, int, int, int],
    frag_size: Tuple[int, int],
    look_range: int
) -> int:
    """Returns how many black frames were found near the ball

    Args:
        image_fragments (np.array): image fragments
        object_coords (tuple): coordinates for object, (x, y, w, h)
        frag_size (tuple): fragments size (y, x)
        look_range (int): how much to extend looking area compared to object size

    Returns:
        int: black_frame_count
    """
    x1 = object_coords[0]
    y1 = object_coords[1] + object_coords[3]
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
            if image_fragments[y][x] == 6:
                black_count += 1
    return black_count