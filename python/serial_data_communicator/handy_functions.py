# This file contains a few neat functions regarding communicatin with serial_com such as get pose

from serial_data_communicator.serial_communicator import serial_com
import yaml
from pathlib import Path
from misc.verbosity_levels import VerboseLevel
import os

class HandyFunctions:
    def __init__(self) -> None:
        # Read all the configs
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        self.load_configs()
       
        if self.verbose_level <= VerboseLevel.DEBUG:
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


    def get_pose(self):
        """Gets the pose and gripper value of the arduino

        Returns:
            tuple: (J1, J2, J3, z, gripper_value)
        """
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} going to request pose")

        print(f"{self.name} not implemented")
        pose = (1,2,3,4,5)

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} got pose: {pose}")

        return pose

    # Maybe add a function to get gripper status and specific joint values etc




handy_functions = HandyFunctions()





