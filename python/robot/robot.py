import traceback
from message.message_types import MessageTypes
import threading
from serial_data_communicator.serial_communicator import serial_com
from serial_data_communicator.handy_functions import handy_functions
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from message.message_updated import MessageUpdated
from math import atan2, cos, sin, pi
import numpy as np
from robot.robot_cmd import RobotCmd
from robot.robot_cmd_types import RobotCmdTypes
import queue

class Robot:
    def __init__(self):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()

        # These are the latest know position of the robot. They're updated as soon as a 
        # move cmd or home cmd is performed
        self.J1 = 0#None #Temp putting these to not None
        self.J2 = 0#None
        self.J3 = 0#None
        x,y = self.forward_kinematics(0,0)
        self.x = x#None 
        self.y = y#None
        self.z = 0#None
        self.gripper_value = 0

        # These are the goal positions, it is where the robot will be when all cmds are 
        # performed, of the robot. These are only used to chain commands.
        self.J1_goal = 0#None #Temp putting these to not None
        self.J2_goal = 0#None
        self.J3_goal = 0#None
        x,y = self.forward_kinematics(0,0)
        self.x_goal = x#None 
        self.y_goal = y#None
        self.z_goal = 0#None
        self.gripper_value_goal = 0


        self.vel = self.config['base_speed']
        self.acc = self.config['base_acceleration']

        # Class that will check if new messages have arrived to serial_communicator
        self.done_event = threading.Event()
        self.message_update = MessageUpdated({MessageTypes.DONE.name: self.done_event}, self.name)

        # Queue of helper RobotCmd, a class that helps when robot should be altered
        self.cmd_queue = queue.Queue() # No max value
        self.cmd_cur:RobotCmd = None

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Inited robot.\nConfig: {self.config},\nand base config: {self.config_base}")

        self.run_thread = threading.Thread(target=self.run, name=self.name + "_thread")
        self.run_thread.daemon = True
        self.run_thread.start()

    def print_pos(self):
        print(f"{self.name}: x: {self.x}, y: {self.y}, z: {self.z}")

    def goto_joints(self, J1, J2, J3, in_rad=True):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move joints to J1: {J1}, J2: {J2}, J3: {J3}, in_rad: {'True' if in_rad else 'False'}")
        
        if not in_rad:
            J1 = np.deg2rad(J1)
            J2 = np.deg2rad(J2)
            J3 = np.deg2rad(J3)

        self.add_move_cmd(J1, J2, J3, self.z_goal, self.gripper_value_goal, self.vel, self.acc)

    def _move_robot(self, J1, J2, J3, z, gripper_value, vel, acc):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move robot to J1: {J1}, J2: {J2}, J3: {J3}, z:{z}, gripper_value:{gripper_value}")

        # Make sure the pos wanted are possible
        success = self.validate_movement_data(J1, J2, J3, z, gripper_value, vel, acc)
        if not success:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name}: Failed with goto_joints")
            return


        # Package the pose in the correct way for the arduino to understand
        data = self.package_data(J1, J2, J3, z, gripper_value, self.vel, self.acc, False, True)

        success = serial_com.send_data(data)

        if not success:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name}: Failed with goto_joints")
            return

        # Wait for robot to be done
        self.done_event.clear()
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
 
    def validate_movement_data(self, J1, J2, J3, z, gripper_value, vel, acc):
        """Validates the input and makes sure the angle values, z-axis values, gripper values,
            velocity and acceleration is within bounds.    
        """

        if not self.config['J1_min'] <= J1 <= self.config['J1_max']:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name} Warning: J1 out of bound")
            return False

        if not self.config['J2_min'] <= J2 <= self.config['J2_max']:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name} Warning: J2 out of bound")
            return False

        if not self.config['J3_min'] <= J3 <= self.config['J3_max']:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name} Warning: J3 out of bound")
            # return False #TODO: temp since J3 isn't with us right now

        if not self.config['z_min'] <= z <= self.config['z_max']:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name} Warning: z out of bound")
            return False

        if not self.config['gripper_min'] <= gripper_value <= self.config['gripper_max']:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name} Warning: gripper_value out of bound")
            return False

        if not self.config['v_min'] <= vel <= self.config['v_max']:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name} Warning: vel out of bound")
            return False

        
        if not self.config['a_min'] <= acc <= self.config['a_max']:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name} Warning: acc out of bound")
            return False

        return True
        
    def forward_kinematics(self,  J1, J2, in_radians=True):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: forward kinematics on: J1:{J1}, J2:{J2}, in radians: {in_radians}")

        L1 = self.config['L1']
        L2 = self.config['L2']
        
        if not in_radians:
            J1 = J1 * pi / 180   # degrees to radians
            J2 = J2 * pi / 180
            
        x = L1 * cos(J1) + L2 * cos(J1 + J2)
        y = L1 * sin(J1) + L2 * sin(J1 + J2)
            
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: resulting positions: x:{x}, y:{y}")

        return x, y

    def inverse_kinematics(self, x, y):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: inverse_kinematics on x:{x}, y:{y}")

        L1 = self.config['L1']
        L2 = self.config['L2']

        
        # Calculate the angle of the second joint. There are always 2 possibilites
        theta2 = [None, None]
        theta2[0] = pi - np.arccos((L1**2 + L2**2 - x**2 - y**2) / (2 * L1 * L2) + 0j) 
        theta2[1] = - theta2[0]
        # Sometimes theta2 becomes complex due to numerical error(I hope). Check if theta2 is too complex to be numerical error
        for i in range(len(theta2)):
            if np.abs(theta2[i].imag) > self.config['imaginary_epsilon']:
                # If theta2 is too imaginary then an invalid position was given, if so put theat2 to None
                theta2[i] = None
            else:
                theta2[i] = theta2[i].real

        # If both theta2 are None then impossible position given
        if theta2[0] is None and theta2[1] is None:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name}: WARNING Invalid position given. Variables were x: {x}, y: {y}, L1: {L1}, L2: {L2}")
            return None, None, None

        def calc_theta1(theta2):
            a = L1 + L2*cos(theta2)
            b = L2*sin(theta2)
            return atan2(y*a-b*x, x*a+b*y)
        
        theta1 = [calc_theta1(theta2[0]), calc_theta1(theta2[1])]
        
        # Calculate "phi" angle so gripper is parallel to the X axis
        phi = pi/2 + theta1[0] + theta2[0] #TODO: not sure how correct this is

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: resulting joints: J1:{theta1}, J2:{theta2}, J3:{phi}")

        return theta1, theta2, phi
        
    def package_data(self, J1, J2, J3, z, gripper_value, vel, acc, home, move, cnvrt_bool=True, in_rad=True):
        """
        cnvrt_bool: The arduino has to receive 1 or 0, not True or False. If home, move and stop need to be translated to
        1 or 0 then put cnvrt_bool to True.
        
        data[0] - STOP 
        data[1] - HOME 
        data[2] - MOVE 
        data[3] - Joint 1 angle
        data[4] - Joint 2 angle
        data[5] - Joint 3 angle
        data[6] - Z position
        data[7] - Gripper value
        data[8] - Speed value
        data[9] - Acceleration value
        """
        #TODO: Implement stop
    
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: going to package data")
        
        if cnvrt_bool:
            home = 1 if home else 0
            move = 1 if move else 0   
        
        # Convert rad to deg, which the arduino understands
        if in_rad:
            J1 = np.rad2deg(J1)
            J2 = np.rad2deg(J2)
            J3 = np.rad2deg(J3)
        
        data = (0,home,move,J1,J2,J3,z,gripper_value,vel,acc)
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: packaged data: {data}")

        return data

    def goto_pos(self, x, y, z):
        """
        Only changes the pos of the robot, not gripper 
        """ 
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move to pos: x:{x}, y:{y}, z:{z}")

        J1,J2,J3 = self.inverse_kinematics(x, y)
        if J1 is None:
            if self.verbose_level <= VerboseLevel.DEBUG:
                print(f"{self.name}: Failed to go to pos: x:{x}, y:{y}, z:{z}")
            return False

        # Pick a valid solution out of the 2 possibile ones
        good_i = None
        for i in range(2):
            if self.validate_movement_data(J1[i], J2[i], J3, z, self.gripper_value, self.vel, self.acc):
                good_i = i
                break
        if good_i is None:
            if self.verbose_level <= VerboseLevel.DEBUG:
                print(f"{self.name}: Failed to go to pos: x:{x}, y:{y}, z:{z}, no possible J1 or J2 angles, J1: {J1}, J2: {J2}")
            return False


        self.add_move_cmd(J1[good_i], J2[good_i], J3, z, self.gripper_value, self.vel, self.acc)
        return True

    def add_robot_cmd(self, type_:RobotCmdTypes, data):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: adding cmd of type: {type_.name}, with data: {data}.")
        
        cmd = RobotCmd(type_, data)
        self.cmd_queue.put(cmd)
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: added cmd, there are {self.cmd_queue.qsize()} cmds in the queue")
        return True

    def add_home_cmd(self):
        return self.add_robot_cmd(RobotCmdTypes.HOME, (None,))

    def add_move_cmd(self, J1, J2, J3, z, gripper_value, vel, acc):
        """This adds a move cmd to the queue. It also updates goal positions.
        Args:
            data (tuple): (J1,J2,J3,z,gripper_value, vel, acc)
        """
        x,y = self.forward_kinematics(J1, J2)
        self.x_goal = x 
        self.y_goal = y 
        self.z_goal = z 
        self.J1_goal = J1 
        self.J2_goal = J2 
        self.J3_goal = J3 
        self.gripper_value_goal = gripper_value

        return self.add_robot_cmd(RobotCmdTypes.MOVE, (J1, J2, J3, z, gripper_value, vel, acc))

    def get_cmds(self):
        """Returns a list of all of the robot's cmds

        Returns:
            [type]: [description]
        """
        cmds = list(self.cmd_queue.queue.copy())
        return (self.cmd_cur, cmds)
        
    def goto_pose(self, J1, J2, J3, z):
        """Changes angles of joints and z

        Args:
            J1 ([type]): [description]
            J2 ([type]): [description]
            J3 ([type]): [description]
            z ([type]): [description]
        """
        if self.verbose_level <= VerboseLevel.DEBUG:            
            print(f"{self.name}: Going to move to pose: J1:{J1}, J2:{J2}, J3:{J3}, z:{z}")

        #TODO: Check if this function is correct. Shouldn't it add a move cmd?
        print(f"{self.name} This function is probably wrong")
        self._move_robot(J1, J2, J3, z, self.gripper_value)

    def get_pose(self):
        return (self.x, self.y, self.z, self.J1, self.J2, self.J3, self.gripper_value)

    def home(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going home.")
        success = self.add_home_cmd()

        if not success and self.verbose_level <= VerboseLevel.WARNING:
            print(f"{self.name}: WARNING Failed to home.")

    def _home(self, *arg):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going home.")
        
        data = self.package_data(0,0,0,0,0,0,0,True,False)
        success = serial_com.send_data(data)

        if not success:
            if self.verbose_level <= VerboseLevel.WARNING:
                print(f"{self.name}: Failed with going home")
            return

        
        self.done_event.clear()
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
        
        self.x_goal = x 
        self.y_goal = y 
        self.z_goal = z 
        self.J1_goal = J1 
        self.J2_goal = J2 
        self.J3_goal = J3 
        self.gripper_value_goal = gripper_value

        if self.verbose_level <= VerboseLevel.WARNING:
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

    def get_x(self):
        return self.x
        
    def get_y(self):
        return self.y
        
    def get_z(self):
        return self.z
        
    def get_J1(self):
        return self.J1
        
    def get_J2(self):
        return self.J2
        
    def get_J3(self):
        return self.J3
        
    def get_gripper(self):
        return self.gripper_value
        
    def get_vel(self):
        return self.vel

    def get_acc(self):
        return self.acc

    def move_x(self, x):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move x to: {x}")

        self.goto_pos(x, self.y, self.z)

    def move_y(self, y):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move y to: {y}")

        self.goto_pos(self.x_goal, y, self.z_goal)

    def move_z(self, z):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move z to: {z}")

        self.goto_pos(self.x_goal, self.y_goal, z)

    def move_xy(self, x, y):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move x to: {x} and y to: {y}")

        self.goto_pos(x, y, self.z_goal)

    def move_J1(self, J1, in_rad=True):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move J1 to: {J1}, in rad: {'True' if in_rad else 'False'}")

        J1 = J1 if in_rad else np.deg2rad(J1)
        self.goto_joints(J1, self.J2_goal, self.J3_goal)
    
    def move_J2(self, J2, in_rad=True):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move J2 to: {J2}, in rad: {'True' if in_rad else 'False'}")

        J2 = J2 if in_rad else np.deg2rad(J2)
        self.goto_joints(self.J1_goal, J2, self.J3_goal)
    
    def move_J3(self, J3, in_rad=True):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to move J3 to: {J3}, in rad: {'True' if in_rad else 'False'}")

        J3 = J3 if in_rad else np.deg2rad(J3)
        self.goto_joints(self.J1_goal, self.J2_goal, J3)
    
    def alter_gripper(self, gripper_value):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: going to change gripper value to: {gripper_value}")

        # Package the pose in the correct way for the arduino to understand
        self.add_move_cmd(self.J1_goal, self.J2_goal, self.J3_goal, self.z_goal, gripper_value, self.vel, self.acc)
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: changed gripper value to: {gripper_value}")
        
    def close_gripper(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: going to close gripper")

        self.alter_gripper(self.config['gripper_max'])

    def open_gripper(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: going to open gripper")

        self.alter_gripper(self.config['gripper_min'])

    def set_velocity(self, vel):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: updating velocity from {self.vel} to {vel}")
        self.vel = vel
        return True

    def set_acceleration(self, acc):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: updating acceleration from {self.acc} to {acc}")
        self.acc = acc
        return True

    def kill(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Dying")    

        
        if self.verbose_level <= VerboseLevel.INFO:
            print(f"{self.name}: Good bye!")
        
    def run(self):
        # A dict of functions to handle the commands
        handle_fns = {RobotCmdTypes.HOME.name: self._home, 
                    RobotCmdTypes.MOVE.name: self._move_robot}
        
        while True:
            if self.verbose_level <= VerboseLevel.DEBUG:
                print(f"{self.name}: Waiting for cmd.")

            self.cmd_cur = self.cmd_queue.get()
            handle_fns[self.cmd_cur.type.name](*self.cmd_cur.data)

            # Done with cmd so put done cmd to None
            self.cmd_cur = None

            

            if self.verbose_level <= VerboseLevel.DEBUG:
                print(f"{self.name}: Done with cmd.\n")



robot = Robot()