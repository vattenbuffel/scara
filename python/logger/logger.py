from misc.verbosity_levels import VerboseLevel
import time
import termcolor
import yaml
from pathlib import Path
import os

class Logger:
    def __init__(self, name, add_timestamp=False):
        self.name = name #String
        self.add_timestamp = add_timestamp #bool
        self.verbose_level = None #mic.verbosityLevels

        # Read all the configs
        self.logger_load_configs()

    def logger_load_configs(self):
        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.logger_config = yaml.load(f, Loader=yaml.FullLoader)
        config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
        with open(config_fp) as f:
            self.config_base = yaml.load(f, Loader=yaml.FullLoader)

        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])

    def print(self, color, msg):
        if self.add_timestamp:
            msg += str(time.time())
        print(termcolor.colored(msg, color)) 

    def ALL(self, msg):
        if self.verbose_level <= VerboseLevel.ALL:
            self.print(self.logger_config['color']["ALL"], msg) 

    def MSG_ARRIVE(self, msg):
        if self.verbose_level <= VerboseLevel.MSG_ARRIVE:
            self.print(self.logger_config['color']["MSG_ARRIVE"], msg) 

    def DEBUG(self, msg):
        if self.verbose_level <= VerboseLevel.DEBUG:
            self.print(self.logger_config['color']["DEBUG"], msg) 

    def INFO(self, msg):
        if self.verbose_level <= VerboseLevel.INFO:
            self.print(self.logger_config['color']["INFO"], msg) 

    def WARNING(self, msg):
        if self.verbose_level <= VerboseLevel.WARNING:
            self.print(self.logger_config['color']["WARNING"], msg) 
    
    def ERROR(self, msg):
        if self.verbose_level <= VerboseLevel.ERROR:
            self.print(self.logger_config['color']["ERROR"], msg) 

