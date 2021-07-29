import cmd
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

class CLI(cmd.Cmd):
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

        if self.verbose_level <= VerboseLevel.INFO:
            print(f"Inited CLI.\nConfig: {self.config},\nand base config: {self.config_base}")

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
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command go home")

        robot.home()
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with command go home")

    def do_pos(self, arg):
        "Moves the robot into the position:  X Y Z"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command pose")

        try:
            robot.move_xyz(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with command pose")

    def do_joints(self, arg):
        "Moves the joints to the angles given:  J1, J2, J3"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command pose")

        try:
            robot.goto_joints(*self.parse(arg), in_rad=False)
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with command pose")

    def do_send(self, arg):
        "Send the data to the robot's arduino. Probably is blocking:  DATA"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command send data")

        serial_com.send_data(arg)
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with send data")

    def do_get_x(self, arg):
        "Prints the x coordinate of where the robot is:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_x")

        robot_x = robot.get_x()
        print(f"{self.prompt} robot_x: {robot_x}")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with  get_x")

    def do_get_y(self, arg):
        "Prints the y coordinate of where the robot is:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_y")

        robot_y = robot.get_y()
        print(f"{self.prompt} robot_y: {robot_y}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with  get_y")

    def do_get_z(self, arg):
        "Prints the z coordinate of where the robot is:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_z")

        robot_z = robot.get_z()
        arduino_z = handy_functions.get_z()
        print(f"{self.prompt} robot_z: {robot_z}, arduino_x: {arduino_z}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with  get_z")

    def do_get_J1(self, arg):
        "Prints the J1 angles:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_J1")

        robot_J1 = np.rad2deg(robot.get_J1())
        arduino_J1 = handy_functions.get_J1(in_rad=False)
        print(f"{self.prompt} robot_J1: {robot_J1}, arduino_J1: {arduino_J1}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with  get_J1")

    def do_get_J2(self, arg):
        "Prints the J2 angle:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_J2")

        robot_J2 = np.rad2deg(robot.get_J2())
        arduino_J2 = handy_functions.get_J2(in_rad=False)
        print(f"{self.prompt} robot_J2: {robot_J2}, arduino_J2: {arduino_J2}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_J2")

    def do_get_J3(self, arg):
        "Prints the J3 angle:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_J3")

        robot_J3 = np.rad2deg(robot.get_J3())
        arduino_J3 = handy_functions.get_J3(in_rad=False)
        print(f"{self.prompt} robot_J3: {robot_J3}, arduino_J3: {arduino_J3}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_J3")

    def do_get_vel(self, arg):
        "Prints the velocity of the robot:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_vel")

        print(f"{self.prompt} Velocity: {robot.get_vel()}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_vel")

            
    def do_get_acc(self, arg):
        "Prints the acceleration of the robot:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_acc")

        print(f"{self.prompt} Acceleration: {robot.get_acc()}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_acc")

    def do_get_gripper(self, arg):
        "Prints the gripper value:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_gripper")

        robot_gripper = robot.get_gripper()
        arduino_gripper = handy_functions.get_gripper()
        print(f"{self.prompt} robot_gripper: {robot_gripper}, arduino_gripper: {arduino_gripper}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_gripper")

    def do_get_pose(self, arg):
        """
        Prints the pose of the robot. First is the python's coordinate of the robot: x, y, z, J1, J2, J3, gripper_value.
        Then the arduinos pose: z, J1, J2, J3, gripper_value:  None
        """
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_pose")

        robot_data = list(robot.get_pose())
        robot_data[3] = np.rad2deg(robot_data[3])
        robot_data[4] = np.rad2deg(robot_data[4])
        robot_data[5] = np.rad2deg(robot_data[5])
        
        arduino = handy_functions.get_pose(in_rad=False)

        print(f"{self.prompt} robot: {robot_data},\n{self.prompt} arduino: {arduino}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_pose")

    def do_move_x(self,arg):
        "Moves the robot only in x:  X"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_x")

        try:
            robot.move_x(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with move_x")

    def do_move_y(self,arg):
        "Moves the robot only in y:  y"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_y")

        try:
            robot.move_y(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with move_y")

    def do_move_z(self,arg):
        "Moves the robot only in z:  z"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_z")

        try:
            robot.move_z(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

    def do_move_xy(self,arg):
        "Moves the robot to the coordinate (x,y):  x, y"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_xy")

        try:
            robot.move_xy(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with move_xy")
    
    def do_moveL_xy(self,arg):
        "Moves the robot to the coordinate (x,y) such that the tcp moves linearly:  x, y"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command moveL_xy")

        try:
            robot.move_xy_line(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with moveL_xy")

    def do_move_J1(self,arg):
        "Moves only the first joint of the robot to the angle given:  J1 [deg]"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_J1")

        try:
            robot.move_J1(*self.parse(arg), in_rad=False)
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with move_J1")

    def do_move_J2(self,arg):
        "Moves only the second joint of the robot to the angle given:  J2 [deg]"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_J2")

        try:
            robot.move_J2(*self.parse(arg), in_rad=False)
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with move_J2")

    def do_move_J3(self,arg):
        "Moves only the third joint of the robot to the angle given:  J3 [deg]"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_J3")

        try:
            robot.move_J3(*self.parse(arg), in_rad=False)
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with move_J1")

    def do_move_gripper(self,arg):
        "Moves only the gripper of the robot to the given value:  val [0-180]"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command move_gripper")

        try:
            robot.alter_gripper(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with move_gripper")
    
    def do_open_gripper(self,arg):
        "Opens the gripper"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command open_gripper")

        try:
            robot.open_gripper()
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with open_gripper")

    def do_close_gripper(self,arg):
        "Closes the gripper"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command close_gripper")

        try:
            robot.close_gripper()
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with close_gripper")

    def do_set_velocity(self,arg):
        "Changes the velocity of all the joints: velocity [0-2000]"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command set_velocity")

        try:
            robot.set_velocity(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with set_velocity")

    def do_set_acceleration(self,arg):
        "Changes the acceleration of all the joints: acceleration [0-4000]"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command set_acceleration")

        try:
            robot.set_acceleration(*self.parse(arg))
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with set_acceleration")

    def do_get_cmds(self, arg):
        "Returns information about the cmds of the robot:"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_cmds")

        try:
            cur_cmd, cmds = robot.get_cmds()
            print(f"{self.prompt} The current cmd is: {cur_cmd}")
            print(f"{self.prompt} There are: {len(cmds)} more waiting and they are and they are:")
            for i, cmd in enumerate(cmds):
                print(f"{self.prompt} {i}: {cmd}")
        except TypeError:
            print(f"{self.prompt} Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_cmds")

    def do_parse(self, arg):
        "Parses a g_code file"
        g_code.self.parse()
        
    def do_move_parse(self, arg):
        "Moves according to self.parsed g_code file"
        g_code.move_self.parsed()

    def do_heatmap_generate(self, arg):
        "Generate a heatmap showing possible x,y coordinates: Show[0 or 1] Save[0 or 1]"

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command heatmap_generate")
        
        try:
            show, save = self.parse(arg)
        except ValueError:
            print(f"{self.prompt}Invalid command. Type help for help")
            return

        heatmap.generate_heatmap(save=save)
        
        if show:
            heatmap.show_heatmap()

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with heatmap_generate")

    def do_heatmap_load_base(self, arg):
        "Loads the base heatmap"

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command heatmap_load_base")
        
        heatmap.load_base_heatmap()
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with heatmap_load_base")

    def do_heatmap_show(self, arg):
        "Shows the current heatmap"

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command heatmap_show")
        
        heatmap.show_heatmap()
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with heatmap_show")

    def do_kill(self, arg):
        "Stops the program and kills all threads but the gui: kill"
        kill()

    def do_EOF(self, arg):
        "Stops the program and kills all threads: ctrl+c"
        self.do_kill(arg)

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
            

def kill():
    if cli.verbose_level <= VerboseLevel.DEBUG:
        print(f"{cli.name}: Killing everything")
    robot.kill()
    serial_com.kill()
    # gui.kill()
    handy_functions.kill()
        
    if cli.verbose_level <= VerboseLevel.INFO:
        print(f"{cli.name}: Good bye!")
    sys.exit()


cli = CLI()

cli_thread = threading.Thread(target=cli.loop, name=cli.name + "_thread")
cli_thread.start()