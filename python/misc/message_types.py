from enum import Enum

class MessageTypes(Enum):
    POS = 0
    DONE = 1

    def str_to_type(string):
        for type in MessageTypes:
            if type.name == string:
                return type
        else:
            raise ValueError("No such received message type as: " + string)

    def __eq__(self, other):
        return self.value == other.value and self.name == other.name
        
