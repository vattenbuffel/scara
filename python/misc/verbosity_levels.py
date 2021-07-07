from enum import Enum
import traceback

class VerboseLevel(Enum):
    ALL = 0
    DEBUG = 1
    WARNING = 2
    ERROR = 3
    MAX = 3

    def str_to_level(str):
        str_ = str.upper()
        for level in VerboseLevel:
            if level.name == str_:
                return level
        
        raise ValueError(f"An invalid verbose level was input. It was: {str}")

    def __lt__(self, other):
        return self.value < other.value

    def __le__(self, other):
        return self.value <= other.value
    
    def __gt__(self, other):
        return self.value > other.value
    
    def __ge__(self, other):
        return self.value <= other.value

    def __eq__(self, other):
        self.value == other.value
        
