from numba import jit


@jit(nopython=True)
def find_black_near_ball(fragments, object_coords, frag_size, look_range):
    """Returns how many black frames were found near the ball"""
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
    if y2 > frag_size[0]:
        y2 = frag_size[0] - 1

    black_count = 0
    for x in range(x1, x2):
        for y in range(y1, y2):
            if fragments[y][x] == 6:
                black_count += 1

    if black_count == 0:
        print(x1, y1, x2, y2)
    return black_count
