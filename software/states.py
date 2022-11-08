from enum import Enum

class State(Enum):
    """State machine enums"""
    Searching = 1
    DriveToBall = 2
    Orbiting = 3
    BallThrow = 4
    Wait = 5
    RemoteControl = 98
    Debug = 99  # state for temporarily testing code