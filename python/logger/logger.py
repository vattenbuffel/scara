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

    def print(self, color, msg, kwargs):
        msg = f"{self.name}: " + msg
        if self.add_timestamp:
            msg += str(time.time())

        print(termcolor.colored(msg, color), **kwargs) 

    def ALL(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.ALL:
            self.print(self.logger_config['color']["ALL"], msg, kwargs) 

    def MSG_ARRIVE(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.MSG_ARRIVE:
            self.print(self.logger_config['color']["MSG_ARRIVE"], msg, kwargs) 

    def DEBUG(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.DEBUG:
            self.print(self.logger_config['color']["DEBUG"], msg, kwargs) 

    def INFO(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.INFO:
            self.print(self.logger_config['color']["INFO"], msg, kwargs) 

    def WARNING(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.WARNING:
            msg = "WARNING " + msg
            self.print(self.logger_config['color']["WARNING"], msg, kwargs) 
    
    def ERROR(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.ERROR:
            msg = "ERROR " + msg
            self.print(self.logger_config['color']["ERROR"], msg, kwargs) 

