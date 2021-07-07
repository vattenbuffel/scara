from enum import Enum

class MessageTypes(Enum):
    POS = 0
    DONE = 1
    MAX = 1

    def str_to_type(self, string):
        for type in MessageTypes:
            if type == string:
                return type
        else:
            raise Exception(f"No such received message type as: {string}")

    def __eq__(self, other):
        return self.value == other.value
        
