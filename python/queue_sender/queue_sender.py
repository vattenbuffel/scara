from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from serial_data_communicator.serial_communicator import serial_com
import threading
from message.message_updated import MessageUpdated
from message.message_types import MessageTypes


class QueueSender:
    """
        Class that allows to queue up messages in the arduino. 
    """
    def __init__(self):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()

        self.n_in_queue = 0
        self.cmd_available_event = threading.Event()
        self.cmd_available_event.set()

        # Class that will check if new messages have arrived to serial_communicator
        self.done_event = threading.Event()
        self.message_update = MessageUpdated({MessageTypes.DONE.name: self.done_event}, self.name)

        self.thread = threading.Thread(target=self.run, name=self.name + "_run_thread")
        self.thread.daemon = True
        self.thread.start()

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Inited HeatMap.\nConfig: {self.config},\nand base config: {self.config_base}")

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

    def send(self, data):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to send data. Currently {self.n_in_queue} data in queue.")

        if self.n_in_queue >= self.config['queue_length']:
            self.cmd_available_event.clear()
       
       # Check if data can be sent
        self.cmd_available_event.wait()
        self.n_in_queue += 1
        success = serial_com.send_data(data)

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Done sending data.")

        return success

    def run(self):
        while True:
            self.done_event.wait()
            if self.verbose_level <= VerboseLevel.DEBUG:
                print(f"{self.name}: Robot done with data. Currently {self.n_in_queue-1} data in queue.")
            self.done_event.clear()
            self.n_in_queue -= 1
            self.cmd_available_event.set()


queue_sender = QueueSender()