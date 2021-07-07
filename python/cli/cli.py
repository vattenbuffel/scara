import cmd
from serial_data_communicator.serial_communicator import serial_com
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os

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
            print(f"Inited serial data sender.\nConfig: {self.config},\nand base config: {self.config_base}")


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
        "Return turtle to the home position:  HOME, not implemented"
        pass

    def do_pos(self, arg):
        "Moves the robot into the position:  X Y Z"
        serial_com.send_pos(parse(*arg))

    def do_send(self, arg):
        "Send the data to the robot's arduino:  DATA"
        serial_com.send_data(arg)


def parse(arg):
    "Convert a series of zero or more numbers to an argument tuple"
    return tuple(map(int, arg.split()))


def kill():
    pass

    
cli = CLI()

# cli.cmdloop()
CLI().cmdloop()