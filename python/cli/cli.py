import cmd
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
        robot.home()

    def do_pos(self, arg):
        "Moves the robot into the position:  X Y Z"
        robot.goto_pose(*parse(arg))

    def do_send(self, arg):
        "Send the data to the robot's arduino:  DATA"
        serial_com.send_data(arg)

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