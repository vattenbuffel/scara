from misc.message_types import MessageTypes
import threading
from serial_data_communicator.serial_communicator import serial_com
from serial_data_communicator.handy_functions import handy_functions
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from message_updated.message_updated import MessageUpdated
from math import atan, acos, sqrt, cos, sin, pi

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
        self.gripper_value = 0

        # Class that will check if new messages have arrived to serial_communicator
        self.done_event = threading.Event()
        self.message_update = MessageUpdated({"DONE": self.done_event})

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Inited robot.\nConfig: {self.config},\nand base config: {self.config_base}")

    def print_pose(self):
        print(f"{self.name}: x: {self.x}, y: {self.y}, z: {self.z}")

    def jog(self, J1, J2, J3, z, gripper_value):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to pose J1: {J1}, J2: {J2}, J3: {J3}, z:{z}, gripper_value:{gripper_value}")

        # Package the pose in the correct way for the arduino to understand
        data = self.package_data(J1, J2, J3, z, gripper_value)

        serial_com.send_data(data)

        # Wait for robot to be done
        self.done_event.wait()
        self.done_event.clear()

        # Update position. 
        x,y = self.forward_kinematics(J1, J2)
        self.x = x
        self.y = y
        self.z = z
        self.J1 = J1
        self.J2 = J2
        self.J3 = J3
        self.gripper_value = gripper_value

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: At pose J1: {J1}, J2: {J2}, J3: {J3}, x:{x}, y:{y}, z:{z}, gripper_value:{gripper_value}")
 
    def alter_gripper(self, gripper_value):
        # Does this function really make sense? If so shouldn't there be a specific function for changing only x, y, z, j1, j2 and j3 as well?
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: going to change gripper value to: {gripper_value}")

        # Package the pose in the correct way for the arduino to understand
        data = self.package_data(self.J1, self.J2, self.J3, self.z, gripper_value)
        serial_com.send_data(data)

        # Wait for robot to be done
        self.done_event.wait()
        self.done_event.clear()

        self.gripper_value = gripper_value

        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: changed gripper value to: {gripper_value}")
        
    def forward_kinematics(self,  J1, J2, in_radians=False):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: forward kinematics on: J1:{J1}, J2:{J2}, in radians: {in_radians}")

        L1 = self.config['L1']
        L2 = self.config['L2']
        
        if not in_radians:
            J1 = J1 * pi / 180   # degrees to radians
            J2 = J2 * pi / 180
            
        x = round(L1 * cos(J1) + L2 * cos(J1 + J2))
        y = round(L1 * sin(J1) + L2 * sin(J1 + J2))
            
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: resulting positions: x:{x}, y:{y}")

        return x, y

    def inverse_kinematics(self, x, y):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: inverse_kinematics on x:{x}, y:{y}")

        L1 = self.config['L1']
        L2 = self.config['L2']
        
        theta2 = acos((sqrt(x) + sqrt(y) - sqrt(L1) - sqrt(L2)) / (2 * L1 * L2))
        if (x < 0 & y < 0): 
            theta2 = (-1) * theta2
        
        
        theta1 = atan(x / y) - atan((L2 * sin(theta2)) / (L1 + L2 * cos(theta2)))
        
        theta2 = (-1) * theta2 * 180 / pi
        theta1 = theta1 * 180 / pi

        # Angles adjustment depending in which quadrant the final tool coordinate x,y is
        if (x >= 0 & y >= 0):        # 1st quadrant
            theta1 = 90 - theta1
        
        if (x < 0 & y > 0):       # 2nd quadrant
            theta1 = 90 - theta1
        
        if (x < 0 & y < 0):        # 3d quadrant
            theta1 = 270 - theta1
            phi = 270 - theta1 - theta2
            phi = (-1) * phi
        
        if (x > 0 & y < 0):        # 4th quadrant
            theta1 = -90 - theta1
        
        if (x < 0 & y == 0): 
            theta1 = 270 + theta1
        
        
        # Calculate "phi" angle so gripper is parallel to the X axis
        phi = 90 + theta1 + theta2
        phi = (-1) * phi

        # Angle adjustment depending in which quadrant the final tool coordinate x,y is
        if (x < 0 & y < 0):        # 3d quadrant
            phi = 270 - theta1 - theta2
        
        if (abs(phi) > 165): 
            phi = 180 + phi

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: resulting joints: J1:{theta1}, J2:{theta2}, J3:{phi}")

        return theta1, theta2, phi
        
    def package_data(self, J1, J2, J3, z, gripper_value):
        """
        data[0] - SAVE button status
        data[1] - RUN button status
        data[2] - Joint 1 angle
        data[3] - Joint 2 angle
        data[4] - Joint 3 angle
        data[5] - Z position
        data[6] - Gripper value
        data[7] - Speed value
        data[8] - Acceleration value
        """
    
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: going to package data")

        data = f"0,1,{J1},{J2},{J3},{z},{gripper_value},{self.config['base_speed']},{self.config['base_acceleration']}"
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: packaged data: {data}")

        return data

    def goto_pose(self, x, y, z, gripper_value):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to pose x: {x}, y: {y}, z: {z}")

        J1,J2,J3 = self.inverse_kinematics(x, y)

        # Package the pose in the correct way for the arduino to understand
        data = self.package_data(J1, J2, J3, z, gripper_value)

        serial_com.send_data(data)

        # Wait for robot to be done
        self.done_event.wait()
        self.done_event.clear()

        # Update position. 
        self.x = x
        self.y = y
        self.z = z
        self.J1 = J1
        self.J2 = J2
        self.J3 = J3
        self.gripper_value = gripper_value

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: At pose J1: {J1}, J2: {J2}, J3: {J3}, x:{x}, y:{y}, z:{z}")

    def get_pose(self):
        return (self.x, self.y, self.z)

    def home(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going home.")

        data = MessageTypes.HOME.name 
        
        serial_com.send_data(data)

        self.done_event.wait()
        self.done_event.clear()

        J1, J2, J3, z, gripper_value = handy_functions.get_pose()

        x, y = self.forward_kinematics(J1, J2)
        self.x = x 
        self.y = y 
        self.z = z 
        self.J1 = J1 
        self.J2 = J2 
        self.J3 = J3 
        self.gripper_value = gripper_value

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: At home, J1: {J1}, J2: {J2}, J3: {J3}, x:{x}, y:{y}, z:{z}")
    
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