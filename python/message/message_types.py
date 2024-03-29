from enum import Enum

class MessageTypes(Enum):
    """Messages that can be received from the arduino

    Args:
        Enum ([type]): [description]

    Raises:
        ValueError: if an invalid string is tried to convert to a MessageType.

    Returns:
        [type]: [description]
    """
    POS = 0
    DONE = 1
    HOME = 2
    HEARTBEAT = 3 # This message will be sent at regular intervals by the arduino and contains all information about it
    READY = 4 # Sent by the arduino when it's ready to receive more data

    def str_to_type(string):
        for type in MessageTypes:
            if type.name == string:
                return type
        else:
            # raise ValueError("No such received message type as: " + string) #temp
            pass

    def __eq__(self, other):
        return self.value == other.value and self.name == other.name
        
