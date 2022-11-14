from enum import Enum

class State(Enum):
    """State machine enums"""
    Stopped = 0
    Searching = 1
    DriveToBall = 2
    Orbiting = 3
    BallThrow = 4
    Wait = 5
    DriveToSearch = 6
    RemoteControl = 98
    Debug = 99  # state for temporarily testing code

class ThrowerState(Enum):
    Off = 0
    StartThrow = 1
    MidThrow = 2
    EndThrow = 3
