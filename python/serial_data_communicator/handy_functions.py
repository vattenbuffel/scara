# This file contains a few neat functions regarding communicatin with serial_com such as get pose

from message.message_types import MessageTypes
from serial_data_communicator.serial_communicator import serial_com
import yaml
from pathlib import Path
from misc.verbosity_levels import VerboseLevel
import os
import threading
from message.message_updated import MessageUpdated
import sys
import numpy as np

class HandyFunctions:
    def __init__(self) -> None:
        # Read all the configs
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        self.load_configs()

        
        # Class that will check if new messages have arrived to serial_communicator
        self.heartbeat_event = threading.Event()
        self.message_update = MessageUpdated({MessageTypes.HEARTBEAT.name: self.heartbeat_event}, self.name)
       
        if self.verbose_level <= VerboseLevel.INFO:
            print(f"Inited serial data communicator.\nConfig: {self.config},\nand base config: {self.config_base}")

    def load_configs(self):
        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
        with open(config_fp) as f:
            self.config_base = yaml.load(f, Loader=yaml.FullLoader)
        
        self.name = self.config['handy_functions_name']
        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])

    def get_pose(self, in_rad=True):
        """Gets the pose and gripper value of the arduino. Waits for a new heartbeat to arrive.

        Returns:
            tuple: (J1, J2, J3, z, gripper_value)
        """
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} going to check pose")

        self.heartbeat_event.clear()
        self.heartbeat_event.wait()

        pose = serial_com.received_messages[MessageTypes.HEARTBEAT.name].data
        pose = [float(p) for p in pose]

        # Convert to rad
        for i in range(3):
            pose[i] = np.deg2rad(pose[i])


        if len(pose) != 5:
            if self.verbose_level <= VerboseLevel.ERROR:
                print(f"{self.name} ERROR no heartbeat received from arduino. Stopping code.")
                exit()

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} got pose: {pose}")

        return pose

            
        
    def get_J1(self, in_rad=True):
        return self.get_pose(in_rad=in_rad)[0]
        
    def get_J2(self, in_rad=True):
        return self.get_pose(in_rad=in_rad)[1]
        
    def get_J3(self, in_rad=True):
        return self.get_pose(in_rad=in_rad)[2]
    
    def get_z(self):
        return self.get_pose()[3]

    def get_gripper(self):
        return self.get_pose()[4]

    def kill(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Dying")   

        
        if self.verbose_level <= VerboseLevel.INFO:
            print(f"{self.name}: Good bye!")


handy_functions = HandyFunctions()





