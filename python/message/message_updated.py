from logger.logger import Logger
import threading
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
import time
from message.message_receieved import MessageReceived
from serial_data_communicator.serial_communicator import serial_com

class MessageUpdated(Logger):
    """
    Class that checks if messages have been updated. It looks at the messages the 
    serial communicator has and compares it to what it has. If a message is found to 
    be never then an event is set. This event is given as an input to this class.
    """
    def __init__(self, events_dict:dict, name, kill_event:threading.Event):
        self.config_base = None # dict
        self.config = None # dict
        self.name = name # str
        self.verbose_level = None # misc.verbosity_level
        self._kill_event = None

        # Read all the configs
        self.load_configs()
        
        # Init the logger
        super().__init__(self.name, self.verbose_level)
        
        self.kill_event = kill_event # Event that will be set when self should be killed


        """
        Dict where the event and message is stored. The key is the string associated 
        with the MessageType. An event is triggered if a newer message is added
        """
        self.updated_dict = {key:[events_dict[key], MessageReceived(key, "")] for key in events_dict}

        # Thread that checks if a newer message has been added to serial com
        self.check_thread = threading.Thread(target=self.loop, name=self.name + "_thread")
        self.check_thread.start()

        self.LOG_INFO(f"Inited serial data communicator.\nConfig: {self.config},\nand base config: {self.config_base}")

    @property
    def temperature(self):
        print("Getting value...")
        return self._temperature

    @temperature.setter
    def temperature(self, value):
        print("Setting value...")
        if value < -273.15:
            raise ValueError("Temperature below -273 is not possible")
        self._temperature = value

    @property
    def kill_event(self):
        return self._kill_event

    @kill_event.setter
    def kill_event(self, event):
        self.LOG_WARNING(f"setting kill event to one with id: {id(event)}")
        if not self._kill_event is None:
            self.LOG_ERROR(f"KILL EVENT ALREADY SET")
        self._kill_event = event


    def load_configs(self):
        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.config = yaml.load(f, Loader=yaml.FullLoader)
        config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
        with open(config_fp) as f:
            self.config_base = yaml.load(f, Loader=yaml.FullLoader)
        
        self.name += '_' + self.config['name']
        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])

    def loop(self):
        while not self.kill_event.is_set():            
            self.LOG_MSG_ARRIVE(f"Checking for new messages")

            for key in self.updated_dict:
                event, msg = self.updated_dict[key]
                # Check if a newer message has arrived, if so set the associated event
                if msg.timestamp < serial_com.received_messages[key].timestamp:
                    self.LOG_MSG_ARRIVE(f"new message arrived of type: {key}")

                    event.set()
                    msg.update(serial_com.received_messages[key].data) # update this class's message

                else:
                    self.LOG_ALL(f"no new message of type: {key}")


            time.sleep(1/self.config['message_update_check_hz'])

