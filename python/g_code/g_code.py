from logger.logger import Logger
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from robot.robot import robot
import numpy as np
from pygcode import Machine, Line
from matplotlib import pyplot as plt
from matplotlib import patches

def in_q2(theta):
    x = np.cos(theta)
    y = np.sin(theta)

    if x < 0:
        if y > 0:
            return True
    return False 
        
def in_q3(theta):
    x = np.cos(theta)
    y = np.sin(theta)

    if x < 0:
        if y < 0:
            return True
    return False

class GCode(Logger):
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

        # Init the logger
        super().__init__(self.name, self.verbose_level)

        self.pos_to_go = [] # List with tuples of goal positions. Populated by a self.parse function        

        self.LOG_INFO(f"Inited GCode.\nConfig: {self.config},\nand base config: {self.config_base}")

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
        self.LOG_DEBUG(f"Going to parse g_code file")

        x_offset = self.config['x_base_offset']
        y_offset = self.config['y_base_offset']
        z_offset = self.config['z_base_offset']

        machine = Machine()
        machine.mode.distance 
        machine.abs_pos.values['X'] = robot.get_x
        machine.abs_pos.values['Y'] = robot.get_y
        machine.abs_pos.values['Z'] = robot.get_z

        with open("./g_code/g_code.gcode",'r') as gcode:
            for line_txt in gcode:
                line = Line(line_txt) 
                if line.block.gcodes:   
                    # If it's a G02 or G03 cmd then it's remade into a series of linear movements
                    if line.gcodes[0].word.value == 2 and line.gcodes[0].word.letter == "G":
                        self.G02_to_G01(line.block.gcodes[0].params, machine.abs_pos.values)
                    elif line.gcodes[0].word.value == 3 and line.gcodes[0].word.letter == "G":
                        self.G03_to_G01(line.block.gcodes[0].params, machine.abs_pos.values)

                    # Move the virtual machine and then move it's position
                    machine.process_gcodes(line.block.gcodes[0]) 
                    x = x_offset + machine.abs_pos.values['X']
                    y = y_offset + machine.abs_pos.values['Y']
                    z = z_offset + machine.abs_pos.values['Z']
                    self.pos_to_go.append((x, y, z))


        self.LOG_DEBUG(f"Done parseing g_code file")

    def G02_to_G01(self, params, cur_pos, show=False):
        start = np.array([cur_pos['X'], cur_pos['Y']])
        center = np.array([params['I'].value, params['J'].value]) + start
        goal = np.array((params['X'].value, params['Y'].value))
        xy = self.arc(start, center, goal, True)
        if show:
            self.show_arc(xy, start, goal, center, True)
        return xy

    def G03_to_G01(self, params, cur_pos, show=False):
        start = np.array([cur_pos['X'], cur_pos['Y']])
        center = np.array([params['I'].value, params['J'].value]) + start
        goal = np.array((params['X'].value, params['Y'].value))
        xy = self.arc(start, center, goal, True)
        if show:
            self.show_arc(xy, start, goal, center, False)
        return xy


    def arc(self, start, center, goal, CW):
        theta1 = np.arctan2(start[1] - center[1], start[0] - center[0])
        theta2 = np.arctan2(goal[1] - center[1], goal[0] - center[0])
        theta2 += 2*np.pi * in_q2(theta1)*in_q3(theta2)
        theta2 -= 2*np.pi * in_q2(theta2)*in_q3(theta1)
        n = abs(int((theta2 - theta1) / self.config['dx']))
        
        theta = np.linspace(theta1, theta2, n)
        r = np.linalg.norm(center-start)
        x = center[0] + r*np.cos(theta) 
        y = center[1] + r*np.sin(theta) 
        
        xy = [(x[i], y[i]) for i in range(n)]
        return xy 


    def show_arc(self, xy_arc, start, goal, center, CW):
        r = np.linalg.norm(center-start)
        theta1 = np.arctan2(start[1] - center[1], start[0] - center[0])
        theta2 = np.arctan2(goal[1] - center[1], goal[0] - center[0])

        if CW:
            theta2_tmp = theta2
            theta2 = theta1
            theta1 = theta2_tmp

        fig, ax = plt.subplots()

        arc = patches.Arc(
            xy=center,
            width=2*r,
            height=2*r,
            angle=0,
            theta1=np.rad2deg(theta1),
            theta2=np.rad2deg(theta2),
            label='arc',
            linewidth=5,
            alpha=0.5,
        )
        ax.add_patch(arc)

        circle = patches.Arc(
            xy=center,
            width=2*r,
            height=2*r,
            angle=0,
            theta1=0,
            theta2=360,
            color='orange',
            label='circle',
        )
        ax.add_patch(circle)

        x = [pos[0] for pos in xy_arc]
        y = [pos[1] for pos in xy_arc]
        plt.plot(x, y, 'green', label='linearised arc', linewidth=5, alpha=0.5)

        ax.set_aspect('equal')
        ax.set(xlim=(-r*1.1 + center[0], r*1.1 + center[0]), ylim=(-r*1.1 + center[1], r*1.1 + center[1]))
        plt.legend()
        plt.show(block=True)


    def move_parsed(self):
        self.LOG_DEBUG(f"Going to move according to parsed g_code file")
        
        # Move to 0, 0, 25 as a good starting spot
        robot.move_xyz(0,0,25)
        # Set the velocity to something nice and slow
        robot.set_velocity(self.config['base_speed'])

        for pos in self.pos_to_go:
            robot.move_xyz(*pos) 
        self.LOG_DEBUG(f"Done moving")







g_code = GCode()
































