# local_planner.py
# ME 134 Team Penguinos
#
# Contains list of states

from enum import Enum

class State(Enum):
    PLAY = 1

    @classmethod
    def default(cls):
        return cls.PLAY