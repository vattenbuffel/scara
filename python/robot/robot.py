from logger.logger import Logger
from message.message_types import MessageTypes
import threading
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
from queue_sender.queue_sender import queue_sender

class Robot(Logger):
    def __init__(self):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()

        # Init the logger
        super().__init__(self.name, self.verbose_level)

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
        self.tcp_vel = self.config['base_tcp_vel']
        self.accuracy = self.config['base_accuracy']

        # Class that will check if new messages have arrived to serial_communicator
        self.done_event = threading.Event()
        self.message_update = MessageUpdated({MessageTypes.DONE.name: self.done_event}, self.name)

        # Queue of helper RobotCmd, a class that helps when robot should be altered
        self.cmd_queue = queue.Queue() # No max value
        self.cmd_cur:RobotCmd = None

        self.LOG_INFO(f"Inited robot.\nConfig: {self.config},\nand base config: {self.config_base}")

        self.run_thread = threading.Thread(target=self.run, name=self.name + "_thread")
        self.run_thread.daemon = True
        self.run_thread.start()

    def print_pos(self):
        print(f"{self.name}: x: {self.x}, y: {self.y}, z: {self.z}")

    def move_J1J2J3(self, J1, J2, J3, in_rad=True):
        self.LOG_DEBUG(f"Going to move joints to J1: {J1}, J2: {J2}, J3: {J3}, in_rad: {'True' if in_rad else 'False'}")
        
        if not in_rad:
            J1 = np.deg2rad(J1)
            J2 = np.deg2rad(J2)
            J3 = np.deg2rad(J3)

        self.add_move_cmd(J1, J2, J3, self.z_goal, self.gripper_value_goal, self.vel, self.vel, self.vel, self.vel, self.acc, self.acc, self.acc, self.acc, self.accuracy)


    def _move_robot(self, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy):
        self.LOG_DEBUG(f"Going to move robot to J1: {J1}, J2: {J2}, J3: {J3}, z:{z}, gripper_value:{gripper_value}")

        # Make sure the pos wanted are possible
        success = self.validate_movement_data(J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc)
        if not success:
            self.LOG_WARNING(f"Failed with move robot")
            return


        # Package the pose in the correct way for the arduino to understand
        data = self.package_data(RobotCmdTypes.MOVE, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy)
        
        # Add msg to queue_sender, block if needed
        success = queue_sender.send(data)

        if not success:
            self.LOG_WARNING(f"Failed with move robot")
            return


        # Update position. 
        x,y = self.forward_kinematics(J1, J2)
        self.x = x
        self.y = y
        self.z = z
        self.J1 = J1
        self.J2 = J2
        self.J3 = J3
        self.gripper_value = gripper_value

        self.LOG_DEBUG(f"At pose J1: {J1}, J2: {J2}, J3: {J3}, x:{x}, y:{y}, z:{z}, gripper_value:{gripper_value}")
 
    def validate_joints_z(self, J1, J2, J3, z):
        if not self.config['J1_min'] <= J1 <= self.config['J1_max']:
            self.LOG_WARNING(f"Warning: J1 out of bound")
            return False

        if not self.config['J2_min'] <= J2 <= self.config['J2_max']:
            self.LOG_WARNING(f"Warning: J2 out of bound")
            return False

        if not self.config['J3_min'] <= J3 <= self.config['J3_max']:
            self.LOG_WARNING(f"Warning: J3 out of bound")
            # return False #TODO: temp since J3 isn't with us right now

        if not self.config['z_min'] <= z <= self.config['z_max']:
            self.LOG_WARNING(f"Warning: z out of bound")
            return False
        
        return True

    def validate_joints_z_vel(self, J1_vel, J2_vel, J3_vel, z_vel):
        if not self.config['v_min'] <= J1_vel <= self.config['v_max']:
            self.LOG_WARNING(f"Warning: J1_vel out of bound")
            return False

        if not self.config['v_min'] <= J2_vel <= self.config['v_max']:
            self.LOG_WARNING(f"Warning: J2_vel out of bound")
            return False

        if not self.config['v_min'] <= J3_vel <= self.config['v_max']:
            self.LOG_WARNING(f"Warning: J3_vel out of bound")
            return False
        
        if not self.config['v_min'] <= z_vel <= self.config['v_max']:
            self.LOG_WARNING(f"Warning: z_vel out of bound")
            return False

        return True

    def validate_joints_z_acc(self, J1_acc, J2_acc, J3_acc, z_acc):
        if not self.config['a_min'] <= J1_acc <= self.config['a_max']:
            self.LOG_WARNING(f"Warning: J1_acc out of bound")
            return False

        if not self.config['a_min'] <= J2_acc <= self.config['a_max']:
            self.LOG_WARNING(f"Warning: J2_acc out of bound")
            return False

        if not self.config['a_min'] <= J3_acc <= self.config['a_max']:
            self.LOG_WARNING(f"Warning: J3_acc out of bound")
            return False

        if not self.config['a_min'] <= z_acc <= self.config['a_max']:
            self.LOG_WARNING(f"Warning: z_acc out of bound")
            return False

        return True

    def validate_gripper(self, gripper_value):
        if not self.config['gripper_min'] <= gripper_value <= self.config['gripper_max']:
            self.LOG_WARNING(f"Warning: gripper_value out of bound")
            return False

        return True

    def validate_movement_data(self, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc):
        """Validates the input and makes sure the angle values, z-axis values, gripper values,
            velocity and acceleration is within bounds.    
        """
        valid = self.validate_joints_z(J1, J2, J3, z)
        valid |= self.validate_joints_z_vel(J1_vel, J2_vel, J3_vel, z_vel)
        valid |= self.validate_joints_z_acc(J1_acc, J2_acc, J3_acc, z_acc)
        valid |= self.validate_gripper(gripper_value)

        return valid
        
    def forward_kinematics(self,  J1, J2, in_radians=True):
        self.LOG_ALL(f"forward kinematics on: J1:{J1}, J2:{J2}, in radians: {in_radians}")

        L1 = self.config['L1']
        L2 = self.config['L2']
        
        if not in_radians:
            J1 = J1 * pi / 180   # degrees to radians
            J2 = J2 * pi / 180
            
        x = L1 * cos(J1) + L2 * cos(J1 + J2)
        y = L1 * sin(J1) + L2 * sin(J1 + J2)
            
        self.LOG_ALL(f"resulting positions: x:{x}, y:{y}")

        return x, y

    def inverse_kinematics(self, x, y):
        self.LOG_ALL(f"inverse_kinematics on x:{x}, y:{y}")

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
            self.LOG_WARNING(f"WARNING Invalid position given. Variables were x: {x}, y: {y}, L1: {L1}, L2: {L2}")
            return None, None, None

        def calc_theta1(theta2):
            a = L1 + L2*cos(theta2)
            b = L2*sin(theta2)
            return atan2(y*a-b*x, x*a+b*y)
        
        theta1 = [calc_theta1(theta2[0]), calc_theta1(theta2[1])]
        
        # Calculate "phi" angle so gripper is parallel to the X axis
        phi = pi/2 + theta1[0] + theta2[0] #TODO: not sure how correct this is

        self.LOG_ALL(f"resulting joints: J1:{theta1}, J2:{theta2}, J3:{phi}")

        return theta1, theta2, phi
        
    def package_data(self, cmd_type:RobotCmdTypes, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy, in_rad=True):
        """
        cnvrt_bool: The arduino has to receive 1 or 0, not True or False. If home, move and stop need to be translated to
        1 or 0 then put cnvrt_bool to True.
        
        data[0] - cmd_type [int] 
        data[1] - Joint 1 angle
        data[2] - Joint 2 angle
        data[3] - Joint 3 angle
        data[4] - Z position
        data[5] - Gripper value
        data[6] - J1_vel value
        data[7] - J2_vel value
        data[8] - J3_vel value
        data[9] - z_vel value
        data[10] - J1_acc value
        data[11] - J2_acc value
        data[12] - J3_acc value
        data[13] - z_acc value
        data[14] - accuracy, how close the arduino has to get to the position before starting to move to the next cmd
        """
        #TODO: Implement stop
    
        self.LOG_ALL(f"going to package data")
        
        # Convert rad to deg, which the arduino understands
        if in_rad:
            J1 = np.rad2deg(J1)
            J2 = np.rad2deg(J2)
            J3 = np.rad2deg(J3)
        
        data = (cmd_type.value,J1,J2,J3,z,gripper_value,J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy)
        
        self.LOG_DEBUG(f"packaged data: {data}")

        return data

    def move_xyz(self, x, y, z):
        """
        Only changes the pos of the robot, not gripper 
        """ 
        self.LOG_DEBUG(f"Going to move to pos: x:{x}, y:{y}, z:{z}")

        J1,J2,J3 = self.inverse_kinematics(x, y)
        if J1 is None:
            self.LOG_WARNING(f"Failed to go to pos: x:{x}, y:{y}, z:{z}")
            return False

        # Pick a valid solution out of the 2 possibile ones
        good_i = None
        for i in range(2):
            if self.validate_joints_z(J1[i], J2[i], J3, z):
                good_i = i
                break
        if good_i is None:
            self.LOG_WARNING(f"Failed to go to pos: x:{x}, y:{y}, z:{z}, no possible J1 or J2 angles, J1: {J1}, J2: {J2}")
            return False


        self.add_move_cmd(J1[good_i], J2[good_i], J3, z, self.gripper_value, self.vel, self.vel, self.vel, self.vel, self.acc, self.acc, self.acc, self.acc, self.accuracy)
        return True

    def moveL_xyz(self, x, y, z):
        """Move the robot so that the tcp moves in a line between current pos and x,y,z

        Args:
            x ([type]): [description]
            y ([type]): [description]
            z ([type]): [description]
        """
        self.LOG_DEBUG(f"Going to move linearly to pos: x:{x}, y:{y} z:{z}")

        d = np.linalg.norm([x-self.x_goal, y-self.y_goal, z-self.z_goal])
        n = int(np.ceil(d / self.config["dx"]))
        xx = np.linspace(self.x_goal, x, n) 
        yy = np.linspace(self.y_goal, y, n) 
        zz = np.linspace(self.z_goal, z, n) 

        self.LOG_DEBUG(f"The distance is: {d} mm, it will take: {n} steps to move with dx: {self.config['dx']}")

        for i in range(n):
            J1,J2,J3 = self.inverse_kinematics(xx[i], yy[i])
            if J1 is None:
                self.LOG_DEBUG(f"Failed to go to pos: xx[i]:{xx[i]}, yy[i]:{yy[i]}, zz[i]:{zz[i]}")
                return False

            # Pick a valid solution out of the 2 possibile ones
            good_j = None
            for j in range(2):
                if self.validate_joints_z(J1[j], J2[j], J3, z):
                    good_j = j
                    break
            if good_j is None:
                self.LOG_DEBUG(f"Failed to go to pos: xx[i]:{xx[i]}, yy[i]:{yy[i]}, zz[i]:{zz[i]}, no possible J1 or J2 angles, J1: {J1}, J2: {J2}")
                return False

            J1_vel, J1_acc, J2_vel, J2_acc, z_vel, z_acc = self.calc_linear_vel_acc(xx[i], yy[i], zz[i], J1[good_j], J2[good_j], self.tcp_vel)
            self.add_move_cmd(J1[good_j], J2[good_j], J3, z, self.gripper_value, J1_vel, J2_vel, self.vel, z_vel, J1_acc, J2_acc, self.acc, z_acc, self.accuracy)
        
        return True

    # Function to calculate how many degrees n steps corresponds to with angle_to_steps = ang_to_steps
    def deg_to_steps_base(self, deg, deg_to_steps, in_rad=True):
        if in_rad:
            deg = np.rad2deg(deg)
        return deg * deg_to_steps

    # Function to calculate deg to steps in theta1
    def deg_to_steps_J1(self, deg, in_rad=True):
        return self.deg_to_steps_base(deg, self.config['J1_angle_to_steps'], in_rad)

    # Function to calculate deg to steps in theta2
    def deg_to_steps_J2(self, deg, in_rad=True):
        return self.deg_to_steps_base(deg, self.config['J2_angle_to_steps'], in_rad)

    # Function to calculate deg to steps in phi
    def deg_to_steps_J3(self, deg, in_rad=True): 
        return self.deg_to_steps_base(deg, self.config['J3_angle_to_steps'], in_rad)

    # Function to calculate how many steps n mm corresponds to in z
    def mm_to_steps_z(self, mm):
        return mm*self.config['z_mm_to_steps']

    def calc_linear_vel_acc(self, x, y, z, J1, J2, tcp_vel):
        self.LOG_DEBUG(f"Going calculate linear joint velocities and accelerations so that J1 and J2 arrive at the same time to: {J1}, {J2} with tcp speed: {tcp_vel}")
        # Doesn't calculate accelerations
        d = np.linalg.norm([x-self.x_goal, y-self.y_goal, z-self.z_goal])
        t = d/tcp_vel
        # If we're already at goal return 0 vel and 0 acc
        if t == 0:
            return 10,10,10,10,10,10

        # TODO: the new velocities should be decreased by a factor which makes them still move linearly
        J1_vel = self.deg_to_steps_J1(np.linalg.norm(J1 - self.J1_goal)) / t
        J2_vel = self.deg_to_steps_J1(np.linalg.norm(J2 - self.J2_goal)) / t
        z_vel = self.deg_to_steps_J1(np.linalg.norm(z - self.z_goal)) / t

        J1_acc, J2_acc, z_acc = self.acc, self.acc, self.acc


        # Clamp the velocities and accelerations
        J1_vel = np.maximum(np.minimum(J1_vel, self.config["v_max"]), 10 + self.config["v_min"])
        J2_vel = np.maximum(np.minimum(J2_vel, self.config["v_max"]), 10 + self.config["v_min"])
        z_vel = np.maximum(np.minimum(z_vel, self.config["v_max"]), 10 + self.config["v_min"])
        J1_acc = np.maximum(np.minimum(J1_acc, self.config["a_max"]), 10 + self.config["a_min"])
        J2_acc = np.maximum(np.minimum(J2_acc, self.config["a_max"]), 10 + self.config["a_min"])
        z_acc = np.maximum(np.minimum(z_acc, self.config["a_max"]), 10 + self.config["a_min"])

        self.LOG_DEBUG(f"Resulting J1_vel: {J1_vel}, J1_acc: {J1_acc}, J2_vel: {J2_vel}, J2_acc: {J2_acc}, z_vel: {z_vel}, z_acc: {z_acc}")
        return J1_vel, J1_acc, J2_vel, J2_acc, z_vel, z_acc

    def moveL_xy(self, x, y):
        """Move the robot so that the tcp moves in a line between current pos and x,y

        Args:
            x ([type]): [description]
            y ([type]): [description]
        """
        self.LOG_DEBUG(f"Going to move linearly to pos: x:{x}, y:{y}")
        self.moveL_xyz(x,y,self.z_goal)

    def add_robot_cmd(self, cmd:RobotCmd):
        self.LOG_DEBUG(f"adding cmd: {cmd}")
        
        self.cmd_queue.put(cmd)
        self.LOG_DEBUG(f"Added cmd, there are {self.cmd_queue.qsize()} cmds in the queue")
        return True

    def add_home_cmd(self):
        cmd = RobotCmd(RobotCmdTypes.HOME, *[-1]*14)
        return self.add_robot_cmd(cmd)

    def add_move_cmd(self, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy):
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

        cmd = RobotCmd(RobotCmdTypes.MOVE, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy)
        return self.add_robot_cmd(cmd)

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
        self.LOG_DEBUG(f"Going home.")
        success = self.add_home_cmd()

        if not success and self.verbose_level <= VerboseLevel.WARNING:
            print(f"{self.name}: WARNING Failed to home.")
        
        return success

    def _home(self, *arg):
        """This adds a home cmd to the robot cmd queue. 
        NOTE: No movement calls should be performed before homeing is done. As this will most likely result in unwanted positions.
        """
        self.LOG_DEBUG(f"Going home.")
        
        data = self.package_data(RobotCmdTypes.HOME,*arg)
        success = queue_sender.send(data)

        if not success:
            self.LOG_WARNING(f"Failed with going home")
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

        self.LOG_INFO(f"At home, J1: {J1}, J2: {J2}, J3: {J3}, x:{x}, y:{y}, z:{z}")
    
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
        self.LOG_DEBUG(f"Going to move x to: {x}")

        return self.move_xyz(x, self.y_goal, self.z_goal)

    def move_y(self, y):
        self.LOG_DEBUG(f"Going to move y to: {y}")

        return self.move_xyz(self.x_goal, y, self.z_goal)

    def move_z(self, z):
        self.LOG_DEBUG(f"Going to move z to: {z}")

        return self.move_xyz(self.x_goal, self.y_goal, z)

    def move_xy(self, x, y):
        self.LOG_DEBUG(f"Going to move x to: {x} and y to: {y}")

        return self.move_xyz(x, y, self.z_goal)

    def move_J1(self, J1, in_rad=True):
        self.LOG_DEBUG(f"Going to move J1 to: {J1}, in rad: {'True' if in_rad else 'False'}")

        J1 = J1 if in_rad else np.deg2rad(J1)
        return self.move_J1J2J3(J1, self.J2_goal, self.J3_goal)
    
    def move_J2(self, J2, in_rad=True):
        self.LOG_DEBUG(f"Going to move J2 to: {J2}, in rad: {'True' if in_rad else 'False'}")

        J2 = J2 if in_rad else np.deg2rad(J2)
        return self.move_J1J2J3(self.J1_goal, J2, self.J3_goal)
    
    def move_J3(self, J3, in_rad=True):
        self.LOG_DEBUG(f"Going to move J3 to: {J3}, in rad: {'True' if in_rad else 'False'}")

        J3 = J3 if in_rad else np.deg2rad(J3)
        return self.move_J1J2J3(self.J1_goal, self.J2_goal, J3)
    
    def alter_gripper(self, gripper_value):
        self.LOG_DEBUG(f"going to change gripper value to: {gripper_value}")

        # Package the pose in the correct way for the arduino to understand
        success = self.add_move_cmd(self.J1_goal, self.J2_goal, self.J3_goal, self.z_goal, gripper_value, self.vel, self.vel, self.vel, self.vel, self.acc, self.acc, self.acc, self.acc)
        

        self.LOG_DEBUG(f"changed gripper value to: {gripper_value}")
        return success

    def close_gripper(self):
        self.LOG_DEBUG(f"going to close gripper")

        self.alter_gripper(self.config['gripper_max'])

    def open_gripper(self):
        self.LOG_DEBUG(f"going to open gripper")

        self.alter_gripper(self.config['gripper_min'])

    def set_velocity(self, vel):
        self.LOG_DEBUG(f"updating velocity from {self.vel} to {vel}")
        self.vel = vel
        return True

    def set_tcp_velocity(self, tcp_vel):
        self.LOG_DEBUG(f"updating tcp velocity from {self.tcp_vel} to {tcp_vel}")
        self.tcp_vel = tcp_vel
        return True

    def set_acceleration(self, acc):
        self.LOG_DEBUG(f"updating acceleration from {self.acc} to {acc}")
        self.acc = acc
        return True

    def kill(self):
        self.LOG_DEBUG(f"Dying")    

        
        self.LOG_INFO(f"Good bye!")
        
    def run(self):
        # A dict of functions to handle the commands
        handle_fns = {RobotCmdTypes.HOME.name: self._home, 
                    RobotCmdTypes.MOVE.name: self._move_robot}
        
        while True:
            self.LOG_DEBUG(f"Waiting for cmd.")

            self.cmd_cur = self.cmd_queue.get()
            handle_fns[self.cmd_cur.type.name](*self.cmd_cur.data())

            # Done with cmd so put done cmd to None
            self.cmd_cur = None

            self.LOG_DEBUG(f"Done with cmd.")



robot = Robot()