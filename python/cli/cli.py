import cmd
from simulator.simulator import simulator
from queue_sender.queue_sender import queue_sender
from logger.logger import Logger
from serial_data_communicator.handy_functions import handy_functions
import threading
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from robot.robot import robot
from serial_data_communicator.serial_communicator import serial_com
import time
import sys
import numpy as np
from g_code.g_code import g_code
from heatmap.heatmap import heatmap
import inputimeout
from misc.kill import kill

class CLI(cmd.Cmd, Logger):
    intro = "Welcome to the Noa's scara robot cli.   Type help or ? to list commands.\n"
    prompt = "(Scara) "
    file = None

    def __init__(self):
        # Init super class
        super(CLI, self).__init__()

        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()
        
        # Init the logger
        Logger.__init__(self, self.name, self.verbose_level)


        # Thread used for handling inputs
        self.run_thread = threading.Thread(target=self.loop, name=self.name + "_thread")
        self.run_thread.setDaemon(True)
        self.run_thread.start()

        self.LOG_INFO(f"Inited CLI.\nConfig: {self.config},\nand base config: {self.config_base}")

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

    def do_home(self, arg):
        "Return robot to the home position:  None"
        self.LOG_DEBUG(f"Received command go home")

        robot.home()
        
        self.LOG_DEBUG(f"Done with command go home")

    def do_move_xyz(self, arg):
        "Moves the robot into the position:  X Y Z"
        self.LOG_DEBUG(f"Received command move_xyz")

        try:
            robot.move_xyz(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with command move_xyz")

    def do_joints(self, arg):
        "Moves the joints to the angles given:  J1, J2, J3"
        self.LOG_DEBUG(f"Received command joints")

        try:
            robot.move_J1J2J3(*self.parse(arg), in_rad=False)
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with command joints")

    def do_send(self, arg):
        "Send the data to the robot's arduino. Probably is blocking:  DATA"
        self.LOG_DEBUG(f"Received command send data")

        serial_com.send_data(arg)
        
        self.LOG_DEBUG(f"Done with send data")

    def do_get_x(self, arg):
        "Prints the x coordinate of where the robot is:  None"
        self.LOG_DEBUG(f"Received command get_x")

        robot_x = robot.get_x()
        print(f"{self.prompt} robot_x: {robot_x}")

        self.LOG_DEBUG(f"Done with  get_x")

    def do_get_y(self, arg):
        "Prints the y coordinate of where the robot is:  None"
        self.LOG_DEBUG(f"Received command get_y")

        robot_y = robot.get_y()
        print(f"{self.prompt} robot_y: {robot_y}")
        
        self.LOG_DEBUG(f"Done with  get_y")

    def do_get_z(self, arg):
        "Prints the z coordinate of where the robot is:  None"
        self.LOG_DEBUG(f"Received command get_z")

        robot_z = robot.get_z()
        arduino_z = handy_functions.get_z()
        print(f"{self.prompt} robot_z: {robot_z}, arduino_x: {arduino_z}")
        
        self.LOG_DEBUG(f"Done with  get_z")

    def do_get_J1(self, arg):
        "Prints the J1 angles:  None"
        self.LOG_DEBUG(f"Received command get_J1")

        robot_J1 = np.rad2deg(robot.get_J1())
        arduino_J1 = handy_functions.get_J1(in_rad=False)
        print(f"{self.prompt} robot_J1: {robot_J1}, arduino_J1: {arduino_J1}")
        
        self.LOG_DEBUG(f"Done with  get_J1")

    def do_get_J2(self, arg):
        "Prints the J2 angle:  None"
        self.LOG_DEBUG(f"Received command get_J2")

        robot_J2 = np.rad2deg(robot.get_J2())
        arduino_J2 = handy_functions.get_J2(in_rad=False)
        print(f"{self.prompt} robot_J2: {robot_J2}, arduino_J2: {arduino_J2}")
        
        self.LOG_DEBUG(f"Done with get_J2")

    def do_get_J3(self, arg):
        "Prints the J3 angle:  None"
        self.LOG_DEBUG(f"Received command get_J3")

        robot_J3 = np.rad2deg(robot.get_J3())
        arduino_J3 = handy_functions.get_J3(in_rad=False)
        print(f"{self.prompt} robot_J3: {robot_J3}, arduino_J3: {arduino_J3}")
        
        self.LOG_DEBUG(f"Done with get_J3")

    def do_get_vel(self, arg):
        "Prints the velocity of the robot:  None"
        self.LOG_DEBUG(f"Received command get_vel")

        print(f"{self.prompt} Velocity: {robot.get_vel()}")
        
        self.LOG_DEBUG(f"Done with get_vel")

            
    def do_get_acc(self, arg):
        "Prints the acceleration of the robot:  None"
        self.LOG_DEBUG(f"Received command get_acc")

        print(f"{self.prompt} Acceleration: {robot.get_acc()}")
        
        self.LOG_DEBUG(f"Done with get_acc")

    def do_get_tcp_vel(self, arg):
        "Prints the tcp vel of the robot:  None"
        self.LOG_DEBUG(f"Received command get_tcp_vel")

        print(f"{self.prompt} Tcp velocity: {robot.get_tcp_vel()}")
        
        self.LOG_DEBUG(f"Done with get_tcp_vel")

    def do_get_gripper(self, arg):
        "Prints the gripper value:  None"
        self.LOG_DEBUG(f"Received command get_gripper")

        robot_gripper = robot.get_gripper()
        arduino_gripper = handy_functions.get_gripper()
        print(f"{self.prompt} robot_gripper: {robot_gripper}, arduino_gripper: {arduino_gripper}")
        
        self.LOG_DEBUG(f"Done with get_gripper")

    def do_get_pose(self, arg):
        """
        Prints the pose of the robot. First is the python's coordinate of the robot: x, y, z, J1, J2, J3, gripper_value.
        Then the arduinos pose: J1, J2, J3, z, gripper_value:  None
        """
        self.LOG_DEBUG(f"Received command get_pose")

        robot_data = list(robot.get_pose())
        robot_data[3] = np.rad2deg(robot_data[3])
        robot_data[4] = np.rad2deg(robot_data[4])
        robot_data[5] = np.rad2deg(robot_data[5])
        
        arduino = handy_functions.get_pose(in_rad=False)

        print(f"{self.prompt} robot: {robot_data},\n{self.prompt} arduino: {arduino}")
        
        self.LOG_DEBUG(f"Done with get_pose")

    def do_move_x(self,arg):
        "Moves the robot only in x:  X"
        self.LOG_DEBUG(f"Received command move_x")

        try:
            robot.move_x(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with move_x")

    def do_move_y(self,arg):
        "Moves the robot only in y:  y"
        self.LOG_DEBUG(f"Received command move_y")

        try:
            robot.move_y(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with move_y")

    def do_move_z(self,arg):
        "Moves the robot only in z:  z"
        self.LOG_DEBUG(f"Received command move_z")

        try:
            robot.move_z(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

    def do_move_xy(self,arg):
        "Moves the robot to the coordinate (x,y):  x, y"
        self.LOG_DEBUG(f"Received command move_xy")

        try:
            robot.move_xy(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with move_xy")
    
    def do_moveL_xyz(self,arg):
        "Moves the robot to the coordinate (x,y,z) such that the tcp moves linearly and with the set tcp velocity:  x, y, z"
        self.LOG_DEBUG(f"Received command moveL_xyz")

        try:
            robot.moveL_xyz(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with moveL_xyz")

    def do_moveL_xy(self,arg):
        "Moves the robot to the coordinate (x,y) such that the tcp moves linearly and with the set tcp velocity:  x, y"
        self.LOG_DEBUG(f"Received command moveL_xy")

        try:
            robot.moveL_xy(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with moveL_xy")

    def do_move_J1(self,arg):
        "Moves only the first joint of the robot to the angle given:  J1 [deg]"
        self.LOG_DEBUG(f"Received command move_J1")

        try:
            robot.move_J1(*self.parse(arg), in_rad=False)
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with move_J1")

    def do_move_J2(self,arg):
        "Moves only the second joint of the robot to the angle given:  J2 [deg]"
        self.LOG_DEBUG(f"Received command move_J2")

        try:
            robot.move_J2(*self.parse(arg), in_rad=False)
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with move_J2")

    def do_move_J3(self,arg):
        "Moves only the third joint of the robot to the angle given:  J3 [deg]"
        self.LOG_DEBUG(f"Received command move_J3")

        try:
            robot.move_J3(*self.parse(arg), in_rad=False)
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with move_J1")

    def do_move_gripper(self,arg):
        "Moves only the gripper of the robot to the given value:  val [0-180]"
        self.LOG_DEBUG(f"Received command move_gripper")

        try:
            robot.alter_gripper(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with move_gripper")
    
    def do_open_gripper(self,arg):
        "Opens the gripper"
        self.LOG_DEBUG(f"Received command open_gripper")

        try:
            robot.open_gripper()
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with open_gripper")

    def do_close_gripper(self,arg):
        "Closes the gripper"
        self.LOG_DEBUG(f"Received command close_gripper")

        try:
            robot.close_gripper()
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with close_gripper")

    def do_set_velocity(self,arg):
        "Changes the velocity of all the joints: J1_vel [deg/s], J2_vel [deg/s], J3_vel [deg/s], z_vel [mm/s]"
        self.LOG_DEBUG(f"Received command set_velocity")

        try:
            robot.set_vels(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with set_velocity")

    def do_set_tcp_velocity(self,arg):
        "Changes the tcp velocity: velocity [mm/s]"
        self.LOG_DEBUG(f"Received command set_tcp_velocity")

        try:
            robot.set_tcp_vel(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with set_tcp_velocity")

    def do_set_acceleration(self,arg):
        "Changes the acceleration of all the joints: J1_acc [deg/(s^2)], J2_acc [deg/(s^2)], J3_acc [deg/(s^2)], z_acc [mm/(s^2)]"
        self.LOG_DEBUG(f"Received command set_acceleration")

        try:
            robot.set_accs(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with set_acceleration")

    def do_get_cmds(self, arg):
        "Returns information about the cmds of the robot:"
        self.LOG_DEBUG(f"Received command get_cmds")

        try:
            cur_cmd, cmds = robot.get_cmds()
            print(f"{self.prompt} The current cmd is: {cur_cmd}")
            print(f"{self.prompt} There are: {len(cmds)} more waiting and they are and they are:")
            for i, cmd in enumerate(cmds):
                print(f"{self.prompt} {i}: {cmd}")
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

        self.LOG_DEBUG(f"Done with get_cmds")

    def do_gcode_load(self, arg):
        "Loads a g_code file"
        g_code.load_gcode(self.prompt)
        
    def do_gcode_parse(self, arg):
        "Parses a g_code file"
        g_code.parse()

    def do_gcode_move(self, arg):
        "Moves according to parsed g_code file"
        g_code.move_parsed()

    def do_gcode_show(self, arg):
        "Shows the resulting image as defined by the gcode file"
        g_code.show(scale=5)

    def do_gcode_loaded(self, arg):
        "Prints the loaded gcode file"
        print(f"{self.prompt}Loaded gcode file: {g_code.get_loaded_gcode_path()}") 

    def do_gcode_parsed(self, arg):
        "Prints True if a gcode has been parsed"
        print(f"{self.prompt}Gcode has been parsed: {g_code.gcode_is_parsed()}") 
    
    def do_gcode_reset(self, arg):
        "Reset the gcode module"
        g_code.reset()
        print(f"{self.prompt}Gcode has been reset") 

    def do_heatmap_generate(self, arg):
        "Generate a heatmap showing possible x,y coordinates: Show[0 or 1] Save[0 or 1]"

        self.LOG_DEBUG(f"Received command heatmap_generate")
        
        try:
            show, save = self.parse(arg)
        except ValueError:
            print(f"{self.prompt}Invalid command. Type help for help")
            return

        heatmap.generate_heatmap(save=save)
        
        if show:
            heatmap.show_heatmap()

        self.LOG_DEBUG(f"Done with heatmap_generate")

    def do_heatmap_load_base(self, arg):
        "Loads the base heatmap"

        self.LOG_DEBUG(f"Received command heatmap_load_base")
        
        heatmap.load_base_heatmap()
        
        self.LOG_DEBUG(f"Done with heatmap_load_base")

    def do_heatmap_show(self, arg):
        "Shows the current heatmap"

        self.LOG_DEBUG(f"Received command heatmap_show")
        
        heatmap.show_heatmap()
        
        self.LOG_DEBUG(f"Done with heatmap_show")
    
    def do_sim_start(self, arg):
        "Starts the simulation animation"
        simulator.plot_start()

    def do_sim_move_xy(self,arg):
        "Simulates the robot to the coordinate (x,y):  x, y"

        try:
            simulator.move_xy(*self.parse(arg))
        except TypeError as e:
            print(f"{self.prompt} Invalid command. Type help for help")

    def do_sim_home(self, arg):
        "Simulates the robot to the home position:  None"
        self.LOG_DEBUG(f"Received command sim go home")

        simulator.home()
        
        self.LOG_DEBUG(f"Done with command sim go home")


    def do_queue_n(self, arg):
        "Prints the number of movement commands currently in arduino's queue"
        self.cli_print(f"N cmds in arduino's queue: {queue_sender.get_n_in_queue()}")

        
    def cli_print(self, data, kwargs): 
        data = self.prompt + data
        print(data, **kwargs) 

    def do_kill(self, arg):
        "Stops the program and kills all threads but the gui: kill"
        kill()

    def do_EOF(self, arg):
        "Stops the program and kills all threads: ctrl+c"
        self.do_kill(arg)

    def kill(self):
        self.LOG_DEBUG(f"Killing everything")
              
        self.LOG_INFO(f"Good bye!")
    
    def loop(self, intro=None):
        # Just a copy of cmd.cmdloop() but with a sleep added
        
        self.preloop()
        if self.use_rawinput and self.completekey:
            try:
                import readline
                self.old_completer = readline.get_completer()
                readline.set_completer(self.complete)
                readline.self.parse_and_bind(self.completekey+": complete")
            except ImportError:
                pass
        try:
            if intro is not None:
                self.intro = intro
            if self.intro:
                self.stdout.write(str(self.intro)+"\n")
            stop = None
            while not stop:
                if self.cmdqueue:
                    line = self.cmdqueue.pop(0)
                else:
                    time.sleep(1/self.config['read_hz'])

                    if self.use_rawinput:
                        try:
                            line = input(self.prompt)
                        except EOFError:
                            line = 'EOF'
                    else:
                        self.stdout.write(self.prompt)
                        self.stdout.flush()
                        line = self.stdin.readline()
                        if not len(line):
                            line = 'EOF'
                        else:
                            line = line.rstrip('\r\n')
                line = self.precmd(line)
                stop = self.onecmd(line)
                stop = self.postcmd(stop, line)
            self.postloop()
        finally:
            if self.use_rawinput and self.completekey:
                try:
                    import readline
                    readline.set_completer(self.old_completer)
                except ImportError:
                    pass

        


    def parse(self, arg):
        "Convert a series of zero or more numbers to an argument tuple"
        try:
            return tuple(map(float, arg.split()))
        except ValueError:
            print(f"{self.prompt} Invalid command. Type help for help")
            




cli = CLI()