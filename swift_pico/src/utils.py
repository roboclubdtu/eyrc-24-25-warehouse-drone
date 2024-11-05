
from enum import Enum

class State(Enum):
    IDLE = 0
    GETTING_PATH = 1
    NAVIGATING = 2