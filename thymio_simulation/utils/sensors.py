from enum import Enum


class SensorTriggered(Enum):
    """
    Enum for sensor triggered
    """
    LEFT = 0
    CENTER_LEFT = 1
    CENTER = 2
    CENTER_RIGHT = 3
    RIGHT = 4
