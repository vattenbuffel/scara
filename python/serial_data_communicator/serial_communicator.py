import serial
import serial.tools.list_ports
from serial.serialutil import SerialException, Timeout
import yaml
from pathlib import Path
import os
from misc.verbosity_levels import VerboseLevel
import traceback
from message.message_receieved import MessageReceived
from message.message_types import MessageTypes
import threading
import time
from logger.logger import Logger

class Communicator(Logger):
    def __init__(self):
        # Read all the configs
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        self.load_configs()

        # Init the logger
        super().__init__(self.name, self.verbose_level)

        # Try to open the usb port
        self.serial = None # serial.Serial
        self.open_serial()
        self.arduino_started = False # Boolean to keep track of if the arduino has started. It will be put to True as soon as a heartbeat arrives

        # Create a dictionary of received messages where the key is the string associated in messages types
        self.received_messages = {type_.name:MessageReceived(type_, "") for type_ in MessageTypes}

        
        # This thread reads messages via the serial port
        self.read_thread = threading.Thread(target=self.receive_message, name="serial_com_receive_msg_thread")
        self.read_thread.daemon = True
        self.read_thread.start()

        # Wait until the first heartbeat arrives and then put arduino_started to True
        self.wait_first_heartbeat(self.received_messages[MessageTypes.HEARTBEAT.name])

        self.LOG_INFO(f"Inited serial data communicator.\nConfig: {self.config},\nand base config: {self.config_base}")

    def wait_first_heartbeat(self, prev_heartbeat):
        """Busy wait loop until a heartbeat has arrived, then put arduino_started to True"""
        self.LOG_DEBUG(f"Waiting for arduino to start")
        
        prev_timestamp = prev_heartbeat.timestamp
        # Wait until a new heartbeat with a different timestamp has arrived
        while True:
            timestamp = self.received_messages[MessageTypes.HEARTBEAT.name].timestamp
            if timestamp != prev_timestamp:
                break
            time.sleep(0.1)

        self.LOG_INFO(f"Arduino started")
        self.arduino_started = True

    def load_configs(self):
        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
        with open(config_fp) as f:
            self.config_base = yaml.load(f, Loader=yaml.FullLoader)
        
        self.name = self.config['serial_com_name']
        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])

    def open_serial(self):
        try:
            # self.serial = serial.Serial(self.config['port'], self.config['baud_rate'], timeout=self.config['timeout_s'])
            self.serial = serial.Serial(self.config['port'], self.config['baud_rate'])
        except SerialException as e:
            if self.verbose_level <= VerboseLevel.ERROR:
                traceback.print_exc()
                print(e)
                self.LOG_ERROR(f"Invalid usb port given. Specify the correct one in the serial sender config file. Possible usb ports are:")
                for port in  list(serial.tools.list_ports.comports()):
                    print(f"{port}\n")
            exit()
        except Exception as e:
            if self.verbose_level <= VerboseLevel.ERROR:
                traceback.print_exc()
                print(e)
            exit()
        
        self.serial.flush()
            
    def add_ending(self, string):
        string += "\r\n"
        return string

    def send_data(self, data, add_ending=True, convert_to_bytes=True):
        """Sends data via the serial port. Returns True if successful otherwhise False

        Args:
            data ([type]): [description]
            add_ending (bool, optional): [description]. Defaults to False.
            convert_to_bytes (bool, optional): [description]. Defaults to True.

        Returns:
            [type]: [description]
        """
        
        self.LOG_DEBUG(f"Going to send data: {data}")
        self.LOG_DEBUG(f"Add ending: {add_ending}")
        self.LOG_DEBUG(f"convert_to_bytes: {convert_to_bytes}")
        
        if not self.arduino_started:
            print(f"Warning: Arduino not started")
            return False
        
        
        # For some reason the msg can't be too long or the arduino will not receive the corret msg. It might also be that 
        # There's a random \n in there somewhere
        data_send = ""
        for d in data:
            data_send += f"{d:.3f},"
        

        # Add \r\n to end of data
        if add_ending:
            data_send = self.add_ending(data_send)
        
        # Converts the data_send to bytes
        if convert_to_bytes:
            data_send = data_send.encode()

        # Split the data into smaller chunks if too big
        data_ = []
        while len(data_send) != 0:
            self.LOG_DEBUG(f"Splitting data into smaller chunks")
            data_.append(data_send[:self.config["serial_buffer_size"]])
            data_send = data_send[self.config["serial_buffer_size"]:]


        # Write the data
        for data in data_:
            self.LOG_DEBUG(f"Sending data: {data}")
            time.sleep(0.1) #TODO: THIS IS NOT GOOD. INSTEAD OF A RANDOM SLEEP DURATION THE CODE SHOULD SOMEHOW WAIT FOR A RESPONSE BEFORE SENDING
            self.serial.write(data)

        
        self.LOG_DEBUG(f"Sent data: {data_send}")
        
        return True

    def kill(self):
        self.LOG_INFO(f" Dying")    

        self.serial.flush()    
        self.serial.close()

    def read_msg(self):
        # Read new data
        try:
            msg = ""
            char = ""
            while char  != "\n":
                while not self.serial.in_waiting: # Busy wait loop. Not neat but the characters should arrive quickly and therefor not block for much
                    pass
                data = self.serial.read()
                char = data.decode()
                msg += char


        except UnicodeDecodeError as e:
            if self.verbose_level <= VerboseLevel.WARNING:
                traceback.print_exc()
                self.LOG_WARNING(f"Invalid char received from the arduino")
            return None

        return msg
            
    def receive_message(self):
        while True:
            # Check if new data
            if self.serial.in_waiting:
                self.LOG_MSG_ARRIVE(f"Data waiting to be read")

                msg = self.read_msg()
                if msg is None:
                    # Invalid char read. Often happens while the arduino is starting
                    continue
                msg_split = msg.split()

                if len(msg_split) == 0:
                    self.LOG_DEBUG("Empty message arrived")
                    continue

                if "\r\n" not in msg:
                    self.LOG_ERROR(f"An unfinished message arrived: {msg}")
                    # raise ValueError(f"{self.name} An unfinished message arrived")



                if not "HEARTBEAT" in msg: 
                    self.LOG_DEBUG(f"Read: {msg}", end="" if "\n" in msg else "\n")

                # See if it is a valid message. A valid message should start with pos or done for example
                try:
                    type_ = MessageTypes.str_to_type(msg_split[0])
                    self.received_messages[msg_split[0].upper()] = MessageReceived(type_, msg_split[1:])

                except ValueError as e:
                    if self.verbose_level <= VerboseLevel.WARNING:
                        print(e)
                        traceback.print_exc()
                        self.LOG_WARNING(f"Invalid message type: {msg_split[0]}")


            else:
                if self.verbose_level <= VerboseLevel.ALL:
                    self.LOG_ALL(f"No data to read")
            
            if not self.serial.in_waiting:
                # Only wait once the buffer has been emptied
                time.sleep(1/self.config['read_hz'])



serial_com = Communicator()
