from enum import Enum

class VerboseLevel(Enum):
    ALL = 0
    MSG_ARRIVE = 1
    DEBUG = 2
    INFO = 2
    WARNING = 3
    ERROR = 4

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
        return self.value == other.value
        
