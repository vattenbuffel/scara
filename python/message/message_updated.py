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
    def __init__(self, events_dict:dict, name):
        # Read all the configs
        self.config_base = None # dict
        self.config = None # dict
        self.name = name # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()

        # Init the logger
        super().__init__(self.name, self.verbose_level)

        """
        Dict where the event and message is stored. The key is the string associated 
        with the MessageType. An event is triggered if a newer message is added
        """
        self.updated_dict = {key:[events_dict[key], MessageReceived(key, "")] for key in events_dict}

        # Thread that checks if a newer message has been added to serial com
        self.check_thread = threading.Thread(target=self.loop, name=self.name + "_thread")
        self.check_thread.daemon = True
        self.check_thread.start()

        if self.verbose_level <= VerboseLevel.INFO:
            print(f"Inited serial data communicator.\nConfig: {self.config},\nand base config: {self.config_base}")


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

    def kill(self):
        self.LOG_DEBUG(f"will be killed")
        
        
        self.LOG_INFO(f"Good bye!")
    
    def loop(self):
        while True:            
            if self.verbose_level <= VerboseLevel.MSG_ARRIVE:
                print(f"{self.name}: Checking for new messages")

            for key in self.updated_dict:
                event, msg = self.updated_dict[key]
                # Check if a newer message has arrived, if so set the associated event
                if msg.timestamp < serial_com.received_messages[key].timestamp:
                    if self.verbose_level <= VerboseLevel.MSG_ARRIVE:
                        print(f"{self.name}: new message arrived of type: {key}")

                    event.set()
                    msg.update(serial_com.received_messages[key].data) # update this class's message

                else:
                    if self.verbose_level <= VerboseLevel.ALL:
                        print(f"{self.name}: no new message of type: {key}")


            time.sleep(1/self.config['message_update_check_hz'])

