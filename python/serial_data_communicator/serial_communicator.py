import serial
import serial.tools.list_ports
from serial.serialutil import SerialException, Timeout
import yaml
from pathlib import Path
import os
from misc.verbosity_levels import VerboseLevel
import traceback
from misc.message_receieved import MessageReceived
from misc.message_types import MessageTypes
import threading
import time

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

        # Create a dictionary of received messages where the key is the string associated in messages types
        self.received_messages = {type_.name:MessageReceived(type_, "") for type_ in MessageTypes}

        
        # This thread reads messages via the serial port
        self.read_thread = threading.Thread(target=self.receive_message)
        self.read_thread.start()

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
        
        self.name = self.config['name']
        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])

    def open_serial(self):
        try:
            self.serial = serial.Serial(self.config['port'], self.config['baud_rate'], timeout=self.config['timeout_s'])
        except SerialException as e:
            if self.verbose_level <= VerboseLevel.ERROR:
                traceback.print_exc()
                print(e)
                print(f"{self.name}: Invalid usb port given. Specify the correct one in the serial sender config file. Possible usb ports are:")
                for port in  list(serial.tools.list_ports.comports()):
                    print(f"{port}\n")
            exit()
        except Exception as e:
            if self.verbose_level <= VerboseLevel.ERROR:
                traceback.print_exc()
                print(e)
            exit()
            
    def add_ending(self, string):
        string += "\r\n"

    def send_data(self, data, add_ending=False):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to send data: {data}")
            print(f"{self.name}: Add ending: {add_ending}")

        # Add \r\n to end of data
        if add_ending:
            self.add_ending(data)
        
        # Write the data
        self.serial.write(data)

        
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Sent data: {data}")

    def kill(self):
        if self.verbose_level <= VerboseLevel.WARNING:
            print(f"Going to kill {self.name}")
        
        self.serial.close()
        if self.verbose_level <= VerboseLevel.ERROR:
            print(f"Good bye from {self.name}")
        
    def receive_message(self):
        while True:
            # Check if new data
            if self.serial.in_waiting:
                if self.verbose_level <= VerboseLevel.DEBUG:
                    print(f"{self.name}: Data waiting to be read")

                # Read new data
                msg = self.serial.readline()
                msg_split = msg.split()

                # See if it is a valid message. A valid message should start with pos or done for example
                try:
                    type_ = MessageTypes.str_to_type(msg_split[0])
                    self.received_messages[msg_split[0].upper()] = MessageReceived(type_, msg_split[1:])
                except KeyError as e:
                    if self.verbose_level <= VerboseLevel.WARNING:
                        print(e)
                        traceback.print_exc()
                        print(f"{self.name}: Invalid message type: {msg_split[0]}")

            else:
                if self.verbose_level <= VerboseLevel.ALL:
                    print(f"{self.name}: No data to read")
            time.sleep(1/self.config['read_hz'])


serial_com = Communicator()
