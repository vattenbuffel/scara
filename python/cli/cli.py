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

        if self.verbose_level <= VerboseLevel.DEBUG:
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
        "Return turtle to the home position:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command go home")

        robot.home()
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with command go home")

    def do_pos(self, arg):
        "Moves the robot into the position:  X Y Z, gripper_value"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command pose")

        try:
            robot.goto_pose(*parse(arg))
        except TypeError:
            print(f"self.prompt Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with command pose")

    def do_jog(self, arg):
        "Moves the robot into the position:  J1, J2, J3, z, gripper_value"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command pose")

        try:
            robot.jog(*parse(arg))
        except TypeError:
            print(f"self.prompt Invalid command. Type help for help")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with command pose")

    def do_send(self, arg):
        "Send the data to the robot's arduino:  DATA"
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
        "Prints the J1 coordinate of where the robot is:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_J1")

        robot_J1 = robot.get_J1()
        arduino_J1 = handy_functions.get_J1()
        print(f"{self.prompt} robot_J1: {robot_J1}, arduino_J1: {arduino_J1}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with  get_J1")


    def do_get_J2(self, arg):
        "Prints the J2 coordinate of where the robot is:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_J2")

        robot_J2 = robot.get_J2()
        arduino_J2 = handy_functions.get_J2()
        print(f"{self.prompt} robot_J2: {robot_J2}, arduino_J2: {arduino_J2}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_J2")


    def do_get_J3(self, arg):
        "Prints the J3 coordinate of where the robot is:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_J3")

        robot_J3 = robot.get_J3()
        arduino_J3 = handy_functions.get_J3()
        print(f"{self.prompt} robot_J3: {robot_J3}, arduino_J3: {arduino_J3}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_J3")


    def do_get_gripper(self, arg):
        "Prints the gripper coordinate of where the robot is:  None"
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Received command get_gripper")

        robot_gripper = robot.get_gripper()
        arduino_gripper = handy_functions.get_gripper()
        print(f"{self.prompt} robot_gripper: {robot_gripper}, arduino_gripper: {arduino_gripper}")
        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done with get_gripper")


    def loop(self, intro=None):
        # Just a copy of cmd.cmdloop() but with a sleep added
        
        self.preloop()
        if self.use_rawinput and self.completekey:
            try:
                import readline
                self.old_completer = readline.get_completer()
                readline.set_completer(self.complete)
                readline.parse_and_bind(self.completekey+": complete")
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

        


def parse(arg):
    "Convert a series of zero or more numbers to an argument tuple"
    return tuple(map(int, arg.split()))

def kill():
    if cli.verbose_level <= VerboseLevel.DEBUG:
        print(f"{cli.name}: will be killed")
    
    print(f"{cli.name}: kill not implemented")
    
    if cli.verbose_level <= VerboseLevel.ERROR:
        print(f"{cli.name}: Good bye!")


cli = CLI()

cli_thread = threading.Thread(target=cli.loop)
cli_thread.start()