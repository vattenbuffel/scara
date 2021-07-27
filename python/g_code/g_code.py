from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from robot.robot import robot
import numpy as np
import re
from pygcode import Machine, Line

class GCode:
    """
        Class that takes a g-code file, interprets it and moves the
        robot to the x,y and z positions set by the g-code file.
    """
    def __init__(self):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()

        self.pos_to_go = [] # List with tuples of goal positions. Populated by a self.parse function        

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Inited GCode.\nConfig: {self.config},\nand base config: {self.config_base}")

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

    def parse(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} Going to parse g_code file")

        with open("./g_code/g_code.gcode",'r') as gcode:
            for line_txt in gcode:
                line = Line(line_txt) 
                if line.block.gcodes:
                    print(line)                                    
                    try:
                        x = line.block.gcodes[0].params['X'].value
                        y = line.block.gcodes[0].params['Y'].value
                        z = line.block.gcodes[0].params['Z'].value
                        self.pos_to_go.append((x, y, z))
                    except:
                        pass

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} Done parseing g_code file")

    def move_parsed(self):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} Going to move according to parsed g_code file")
        for pos in self.pos_to_go:
            robot.move_xyz(*pos) 

g_code = GCode()
































