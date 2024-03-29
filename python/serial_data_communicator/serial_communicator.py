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

        # When a done message arrives a callback can be performed. This is the list of callbacks
        self.done_callbacks = [] # List of callback functions
        
        # This thread reads messages via the serial port
        self.read_thread = threading.Thread(target=self.receive_message, name="serial_com_receive_msg_thread")
        self.kill_event = threading.Event()
        self.kill_event.clear()
        self.arduino_ready_lock = threading.Lock() # Event that's set when arduino is ready to receive data, when it sends self.config["ready_to_read_str"]
        self.arduino_ready_event = threading.Event() # Lock to keep the reading and writing thread from being fucky wucky with the above event
        self.read_thread.start()

        # Wait until the first heartbeat arrives and then put arduino_started to True
        self.wait_first_heartbeat(self.received_messages[MessageTypes.HEARTBEAT.name])

        self.LOG_INFO(f"Inited serial data communicator.\nConfig: {self.config},\nand base config: {self.config_base}")
    
    def add_done_callback(self, fn):
        self.done_callbacks.append(fn)

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
        
        self.LOG_DEBUG(f"Going to send data: {data}, add ending: {add_ending}, convert_to_bytes: {convert_to_bytes}")
        
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
            self.LOG_ALL(f"Splitting data into smaller chunks")
            data_.append(data_send[:self.config["serial_buffer_size"]])
            data_send = data_send[self.config["serial_buffer_size"]:]


        # Write the data
        for data in data_:
            self.arduino_ready_event.wait()
            self.LOG_DEBUG(f"Sending data: {data}")
            self.arduino_ready_lock.acquire()
            self.serial.write(data)
            self.arduino_ready_event.clear()
            self.arduino_ready_lock.release()

        return True

    def kill(self):
        self.LOG_INFO(f"Dying")    

        self.kill_event.set()
        self.read_thread.join()
        self.serial.flush()    
        self.serial.close()

    def read_msg(self):
        # Read new data
        try:
            msg = ""
            char = ""
            while char  != "\n":
                while not self.serial.in_waiting: 
                    time.sleep(1/10000)
                data = self.serial.read()
                char = data.decode()
                msg += char


        except UnicodeDecodeError as e:
            if self.verbose_level <= VerboseLevel.WARNING:
                traceback.print_exc()
                self.LOG_WARNING(f"Invalid char received from the arduino")
            return None

        return msg
            
    def process_received_msg(self, msg):
        if msg is None:
            # Invalid char read. Often happens while the arduino is starting
            return False
        msg_split = msg.split()

        if len(msg_split) == 0:
            self.LOG_DEBUG("Empty message arrived")
            return False

        if "\r\n" not in msg:
            self.LOG_ERROR(f"An unfinished message arrived: {msg}")
            # raise ValueError(f"{self.name} An unfinished message arrived")
            return False

        if not "HEARTBEAT" in msg: 
            self.LOG_DEBUG(f"Read: {msg}", end="" if "\n" in msg else "\n")

        # See if the arduino is ready to receive
        if self.config["ready_to_read_str"] == msg.strip("\r\n"):
            self.arduino_ready_lock.acquire()
            self.arduino_ready_event.set()
            self.arduino_ready_lock.release()

        return msg_split

    def receive_message(self):
        while not self.kill_event.is_set():
            # Check if new data
            if self.serial.in_waiting:
                self.LOG_MSG_ARRIVE(f"Data waiting to be read")

                msg = self.read_msg()
                msg_split = self.process_received_msg(msg)
                if msg_split == False:
                    continue

                # See if it is a valid message. A valid message should start with pos or done for example
                try:
                    type_ = MessageTypes.str_to_type(msg_split[0])
                    self.received_messages[msg_split[0].upper()] = MessageReceived(type_, msg_split[1:])

                    # If the message signals that the arduino is done, call all callbacks
                    if type_ is not None and type_ == MessageTypes.DONE:
                        for fn in self.done_callbacks:
                            fn()

                except ValueError as e:
                    if self.verbose_level <= VerboseLevel.WARNING:
                        print(e)
                        traceback.print_exc()
                        self.LOG_WARNING(f"Invalid message type: {msg_split[0]}")

            else:
                self.LOG_ALL(f"No data to read")
            
            if not self.serial.in_waiting:
                # Only wait once the buffer has been emptied
                time.sleep(1/self.config['read_hz'])

        self.LOG_INFO(f"Killing thread: {self.read_thread.name}")


serial_com = Communicator()
