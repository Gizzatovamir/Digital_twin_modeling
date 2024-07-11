from enum import IntEnum


class State(IntEnum):
    UNDEF = 0
    VALID = 1
    INVALID = 2

    def __str__(self):
        return f"{self.value}"


class ChildType(IntEnum):
    TRAIN = 0
    TILE = 1
