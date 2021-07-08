import threading
from serial_data_communicator.serial_communicator import serial_com
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from message_updated.message_updated import MessageUpdated

class Robot:
    def __init__(self):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()

        self.x = None
        self.y = None
        self.z = None
        self.J1 = None
        self.J2 = None
        self.J3 = None

        # Class that will check if new messages have arrived to serial_communicator
        self.done_event = threading.Event()
        self.message_update = MessageUpdated({"DONE": self.done_event})

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Inited robot.\nConfig: {self.config},\nand base config: {self.config_base}")

    def print_pose(self):
        print(f"{self.name}: x: {self.x}, y: {self.y}, z: {self.z}")

    def goto_pose(self, x, y, z):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to pose x: {x}, y: {y}, z: {z}\nNOT IMPLEMENTED YET")

        # Package the pose in the correct way for the arduino to understand
        data = f"{self.name}: xyz:{x},{y},{z}"

        serial_com.send_data(data)

        # Maybe wait for a response before updating the positions here, unsure
        self.x = x
        self.y = y
        self.z = z

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: At pose x: {x}, y: {y}, z: {z}")

    def get_pose(self):
        return (self.x, self.y, self.z)

    def home(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going home\nNOT IMPLEMENTED YET")

        data = "" #TODO: Change this to the correct data
        self.x = 0 #TODO: Change this to the correct data
        self.y = 0 #TODO: Change this to the correct data
        self.z = 0 #TODO: Change this to the correct data
        self.J1 = 0 #TODO: Change this to the correct data
        self.J2 = 0 #TODO: Change this to the correct data
        self.J3 = 0 #TODO: Change this to the correct data
        
        # serial_com.send_data() # TODO: Send some data so that it goes home

        self.done_event.wait()

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: At home")
    
    def load_configs(self):
        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
        with open(config_fp) as f:
            self.config_base = yaml.load(f, Loader=yaml.FullLoader)
        
        self.name = self.config['name']
        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])


robot = Robot()