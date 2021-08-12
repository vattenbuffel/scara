from misc.verbosity_levels import VerboseLevel
import time
import termcolor
import yaml
from pathlib import Path
import os

class Logger:
    def __init__(self, name, verbose_level:VerboseLevel, add_timestamp=False):
        self.name = name #String
        self.add_timestamp = add_timestamp #bool
        self.verbose_level = verbose_level #mic.verbosityLevels

        # Read all the configs
        self.logger_load_configs()

    def logger_load_configs(self):
        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.logger_config = yaml.load(f, Loader=yaml.FullLoader)
            
    def LOG(self, color, msg, kwargs):
        msg = f"{self.name}: " + msg
        if self.add_timestamp:
            msg += str(time.time())

        print(termcolor.colored(msg, color), **kwargs) 

    def LOG_ALL(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.ALL:
            self.LOG(self.logger_config['color']["ALL"], msg, kwargs) 

    def LOG_MSG_ARRIVE(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.MSG_ARRIVE:
            self.LOG(self.logger_config['color']["MSG_ARRIVE"], msg, kwargs) 

    def LOG_DEBUG(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.DEBUG:
            self.LOG(self.logger_config['color']["DEBUG"], msg, kwargs) 

    def LOG_INFO(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.INFO:
            self.LOG(self.logger_config['color']["INFO"], msg, kwargs) 

    def LOG_WARNING(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.WARNING:
            msg = "WARNING " + msg
            self.LOG(self.logger_config['color']["WARNING"], msg, kwargs) 
    
    def LOG_ERROR(self, msg, **kwargs):
        if self.verbose_level <= VerboseLevel.ERROR:
            msg = "ERROR " + msg
            self.LOG(self.logger_config['color']["ERROR"], msg, kwargs) 

