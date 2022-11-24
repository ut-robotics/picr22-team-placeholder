from enum import Enum


class State(Enum):
    """State machine enums"""
    Stopped = 0
    Searching = 1
    DriveToBall = 2
    Orbiting = 3
    BallThrow = 4
    EscapeFromBasket = 5
    RemoteControl = 98
    Debug = 99  # state for temporarily testing code


class ThrowerState(Enum):
    Off = 0
    StartThrow = 1
    MidThrow = 2
    EndThrow = 3


class SearchState(Enum):
    Off = 0
    StartSearch = 1
    Left = 2
    Right = 3
    DriveToSearch = 4


class EscapeState(Enum):
    Off = 0
    StartEscape = 1
    Reverse = 2
    TurningFromBasket = 3
    DrivingAway = 4

class OrbitDirection(Enum):
    Right = -1
    Left = 1