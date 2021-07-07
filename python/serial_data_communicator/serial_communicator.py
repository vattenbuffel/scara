import serial
import serial.tools.list_ports
from serial.serialutil import SerialException
import yaml
from pathlib import Path
import os
from misc.verbosity_levels import VerboseLevel
import traceback

class Communicator:
    def __init__(self):
        # Read all the configs
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        self.load_configs()

        # Try to open the usb port
        self.serial = None # serial.Serial
        self.open_serial()



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

    def open_serial(self):
        try:
            self.serial = serial.Serial(self.config['port'], self.config['baud_rate'])
        except SerialException as e:
            if self.verbose_level <= VerboseLevel.ERROR:
                traceback.print_exc()
                print(e)
                print(f"Invalid usb port given. Specify the correct one in the serial sender config file. Possible usb ports are:")
                for port in  list(serial.tools.list_ports.comports()):
                    print(f"{port}\n")
            exit()
        except Exception as e:
            if self.verbose_level <= VerboseLevel.ERROR:
                traceback.print_exc()
                print(e)
            exit()
            
    def send_pos(self, x, y, z, add_ending=False):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Going to send pos x: {x}, y: {y}, z: {z}\nNOT IMPLEMENTED YET")
        
        data = f"xyz:{x},{y},{z}"

        self.send_data(data)

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Sent pos x: {x}, y: {y}, z: {z}")
        

    def add_ending(self, string):
        string += "\r\n"

    def send_data(self, data, add_ending=False):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Going to send data: {data}")
            print(f"Add ending: {add_ending}")

        # Add \r\n to end of data
        if add_ending:
            self.add_ending(data)
        
        # Write the data
        self.serial.write(data)

        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Sent data: {data}")





serial_com = Communicator()