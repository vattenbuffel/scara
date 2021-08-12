from logger.logger import Logger
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from serial_data_communicator.serial_communicator import serial_com
import threading
from message.message_updated import MessageUpdated
from message.message_types import MessageTypes


class QueueSender(Logger):
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

        # Init the logger
        super().__init__(self.name, self.verbose_level)

        self.n_in_queue = 0
        self.n_in_queue_lock = threading.Lock()
        self.cmd_available_event = threading.Event()
        self.cmd_available_event.set()

        # Whenever the arduino is done, a callback should be called to increase done_counter to ensure no dones are missed
        serial_com.add_done_callback(self.done_callback)
        self.done_lock = threading.Lock()
        self.done_counter = 0

        self.done_event = threading.Event()

        self.thread = threading.Thread(target=self.run, name=self.name + "_run_thread")
        self.thread.daemon = True
        self.thread.start()

        self.LOG_INFO(f"Inited HeatMap.\nConfig: {self.config},\nand base config: {self.config_base}")

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

    def done_callback(self):
        """Whenever done has been received by serial com, this function should be called
        """
        with self.done_lock:
            self.done_counter+=1
            self.done_event.set()

    def send(self, data):
        self.LOG_DEBUG(f"Going to send data. Currently {self.n_in_queue} data in queue.")

        if self.n_in_queue >= self.config['queue_length']:
            self.cmd_available_event.clear()
       
       # Check if data can be sent
        self.cmd_available_event.wait()
        with self.n_in_queue_lock:
            self.n_in_queue += 1
        success = serial_com.send_data(data)

        self.LOG_DEBUG(f"Done sending data.")

        return success

    def get_n_in_queue(self):
        with self.n_in_queue_lock:
            return self.n_in_queue

    def run(self):
        while True:
            self.done_event.wait()
            self.LOG_DEBUG(f"Robot done with data. Currently {self.n_in_queue-1} data in queue.")
            with self.n_in_queue_lock:
                self.n_in_queue -= 1
            self.cmd_available_event.set()
            
            with self.done_lock:
                self.done_counter-=1
                
                # If all of the dones have been processed
                if self.done_counter == 0:
                    self.done_event.clear()


queue_sender = QueueSender()