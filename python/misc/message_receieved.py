from misc.message_types import MessageTypes
import time

class MessageReceived:
    def __init__(self, type:MessageTypes, data:str):
        self.type = type
        self.data = data
        self.timestamp = time.time()


    def __eq__(self, other):
        return self.type == other.type and self.data == other.data and self.timestamp == other.timestamp