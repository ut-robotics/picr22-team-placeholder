# File for helper functions
import os


def map_range(x, in_min, in_max, out_min, out_max):
    """Arduino map from https://stackoverflow.com/questions/70643627/python-equivalent-for-arduinos-map-function

    Args:
        x (int): Current value
        in_min (int): Minimum input value
        in_max (int): Maximum input value
        out_min (int): Minimum output value
        out_max (int): Maximum output value

    Returns:
        int: New value mapped to output range
    """
    return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min


def get_colors_pkl_path():
    """Returns the current colors.pkl path.

    Returns:
        string: colors.pkl path
    """
    pkl_path = "colors/colors.pkl"
    if not os.path.exists(pkl_path) and os.path.exists(os.path.join("software", pkl_path)):
        pkl_path = os.path.join("software", pkl_path)
    return pkl_path

# 0.375,834,919.3,270.4375 - it physically cannot hit from this distance
def calculate_throw_speed(basket_dist):
    """Calculates throw speed based on basket distance 

    Args:
        basket_dist (float): Distance to basket

    Returns:
        int: ThrowerSpeed
    """
    # Values calibrated using linear regression
    return int(basket_dist * 0.1242374809693249 + 726.840208932238)