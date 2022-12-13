# File for helper functions
import os
import tomli


def map_range(x: int, in_min: int, in_max: int, out_min: int, out_max: int) -> int:
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


def get_colors_pkl_path() -> str:
    """Returns the current colors.pkl path.

    Returns:
        string: colors.pkl path
    """
    pkl_path = "colors/colors.pkl"
    if not os.path.exists(pkl_path) and os.path.exists(os.path.join("software", pkl_path)):
        pkl_path = os.path.join("software", pkl_path)
    return pkl_path

def calculate_throw_speed(basket_dist: float) -> int:
    """Calculates throw speed based on basket distance 

    Args:
        basket_dist (float): Distance to basket

    Returns:
        int: ThrowerSpeed
    """
    # Values calibrated using linear regression
    # TODO - recalibrate for new robot
    return int(basket_dist * 0.12429200986031244 + 681.6616520569161) + 15 # mix of both wednesday measurements

def load_config() -> dict:
    """Returns the config data

    Returns:
        dict: configuration
    """
    config_path = "config.toml"
    if not os.path.exists(config_path) and os.path.exists(os.path.join("software", config_path)):
        config_path = os.path.join("software", config_path)
    with open(config_path, "rb") as f:
        config = tomli.load(f)
    return config
