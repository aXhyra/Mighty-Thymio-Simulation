from enum import Enum


class ThymioStates(Enum):
    FIRST_HALF = 0
    SECOND_HALF = 1


class ThymioWallStates(Enum):
    NO_WALL = 0
    WALL_L = 1
    WALL_R = 2
    ROTATE_TO_WALL = 3
    DONE_ROTATE = 4
    ROTATE_AWAY = 5
    MOVING_AWAY = 6
    DONE = 7
