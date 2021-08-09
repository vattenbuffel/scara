from logger.logger import Logger
from PIL import Image, ImageDraw, ImageOps
import glob
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

        self.gcode_file_path = None # Str with name of parsed file

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

    def chose_file(self, paths, prompt=""):
        """Given a list of paths, will prompt the user for which to chose and then return that

        Args:
            paths ([type]): [description]
            prompt (str, optional): [description]. Defaults to "".

        Raises:
            IndexError: [description]

        Returns:
            [type]: [description]
        """
        print(f"{prompt}What g_code file would you like to chose?")
        for i, path in enumerate(paths):
            print(f"{prompt}[{i}]: {path}")

        chosen_i = input(f"{prompt}File nr: ") 
        try:        
            chosen_i = int(chosen_i)
        except ValueError:
            self.LOG_ERROR(f"Invalid input. A number must be input and you input: {chosen_i}")
            return None

        try:
            if chosen_i < 0:
                raise IndexError
            path = paths[chosen_i]
        except IndexError:
            self.LOG_ERROR(f"Invalid file chosen. Max index: {len(paths)}, min index: 0, chosen index: {chosen_i}")
            return None
        
        return path

    def get_possible_paths(self, prompt=""):
        """Returns a list of all possible gcode file names

        Args:
            prompt (str, optional): [description]. Defaults to "".
        """
        # All files ending with .gcode
        paths = glob.glob(self.config['base_path'] + "*" + ".gcode")

        if len(paths) == 0:
            print(f"{prompt}There are no gcode files to load. If you know that you have files, make sure they're in: {self.config['base_path']}")
            return None
        
        return paths

    def load_gcode(self, prompt=""):
        self.LOG_DEBUG(f"load_gcode")

        paths = self.get_possible_paths(prompt=prompt)
        if paths is None:
            return False

        path = self.chose_file(paths, prompt=prompt)
        if path is None:
            return False

        self.set_gcode_file(path)
        if self.gcode_file_path is False:
            return False
        return True

    def set_gcode_file(self, path):
        """Sets the g_code file which then can be parsed

        Args:
            path ([type]): [description]
        """
        self.LOG_DEBUG(f"Setting gcode file to: {path}")
        self.gcode_file_path = path

    def get_loaded_gcode_path(self):
        return self.gcode_file_path

    def gcode_is_parsed(self):
        """Returns True if a gocde file has been parsed, ie there are stored movements or false if not
        """
        return len(self.pos_to_go) > 0

    def reset(self):
        """Clears all loaded file names and positions, essentially reseting the g_code module
        """

        self.gcode_file_path = None
        self.pos_to_go = []

    def show(self):
        """Creates an image of what the gcode will result in and shows it
        """
        self.LOG_DEBUG(f"show with file: {self.gcode_file_path}")

        if not self.gcode_is_parsed:
            self.LOG_WARNING(f"No gcode file has been parsed")
            return False

        pos_to_go = np.array(self.pos_to_go)
        good_pos = pos_to_go[np.logical_or(pos_to_go[:,2] == self.config['draw_height'], pos_to_go[:,2] == self.config['move_height'])]

        width = good_pos[:,0].max() - good_pos[:,0].min()
        x_offset = width*0.05
        width += 2*x_offset
        width = int(width)
        x_offset -= good_pos[:,0].min()

        height = good_pos[:,1].max() - good_pos[:,1].min()
        y_offset = height*0.05
        height += 2*y_offset
        height = int(height)
        y_offset -= good_pos[:,1].min()
        offset = np.array([x_offset, y_offset])

        white = (255,255,255)
        black = (0,0,0)
        img = Image.new('RGB', (width,height), color = white)
        draw = ImageDraw.Draw(img)
        
        cur_pos = good_pos[0]

        for pos in good_pos:
            if pos[2] == self.config['draw_height'] and cur_pos[2] == self.config['draw_height']:
                start = cur_pos[:2] + offset
                end = pos[:2] + offset
                draw.line([tuple(start), tuple(end)], fill=black, width=3)

            cur_pos = pos

        # Flip the img to correct for the inverted y coordinates
        img = ImageOps.flip(img)

        img.show()

        return True

    def parse(self):
        self.LOG_DEBUG(f"Going to parse g_code with name: {self.gcode_file_path}")

        if self.gcode_file_path is None:
            self.LOG_WARNING(f"No gcode file has been loaded")
            return False

        x_offset = self.config['x_base_offset']
        y_offset = self.config['y_base_offset']
        z_offset = self.config['z_base_offset']

        machine = Machine()
        machine.mode.distance 
        machine.abs_pos.values['X'] = robot.get_x
        machine.abs_pos.values['Y'] = robot.get_y
        machine.abs_pos.values['Z'] = robot.get_z

        with open(self.gcode_file_path,'r') as gcode:
            for line_txt in gcode:
                line = Line(line_txt) 
                if line.block.gcodes:   
                    # If it's a G02 or G03 cmd then it's remade into a series of linear movements
                    if line.gcodes[0].word.value == 2 and line.gcodes[0].word.letter == "G":
                        self.G02_to_G01(line.block.gcodes[0].params, machine.abs_pos.values)
                    elif line.gcodes[0].word.value == 3 and line.gcodes[0].word.letter == "G":
                        self.G03_to_G01(line.block.gcodes[0].params, machine.abs_pos.values)

                    # Only move if an accepted g_code
                    if line.block.gcodes[0].word.letter == "G" and 0<=line.block.gcodes[0].word.value<=4:
                        # Move the virtual machine and then move it's position
                        machine.process_gcodes(line.block.gcodes[0]) 
                        x = x_offset + machine.abs_pos.values['X']
                        y = y_offset + machine.abs_pos.values['Y']
                        z = z_offset + machine.abs_pos.values['Z']
                        self.pos_to_go.append((x, y, z))


        # Verify that all the positions are valid
        for pos in self.pos_to_go:
            if not robot.validate_pos(*pos):
                self.LOG_ERROR(f"Invalid position in gcode file. Resetting gcode module.")
                self.reset()
                return False

        self.LOG_DEBUG(f"Done parsing g_code file")
        return True

    def G02_to_G01(self, params, cur_pos, show=False):
        start = np.array([cur_pos['X'], cur_pos['Y']])
        center = np.array([params['I'].value, params['J'].value]) + start
        goal = np.array((params['X'].value, params['Y'].value))
        xy = self.arc(start, center, goal)
        if show:
            self.show_arc(xy, start, goal, center, True)
        return xy

    def G03_to_G01(self, params, cur_pos, show=False):
        start = np.array([cur_pos['X'], cur_pos['Y']])
        center = np.array([params['I'].value, params['J'].value]) + start
        goal = np.array((params['X'].value, params['Y'].value))
        xy = self.arc(start, center, goal)
        if show:
            self.show_arc(xy, start, goal, center, False)
        return xy


    def arc(self, start, center, goal):
        theta1 = np.arctan2(start[1] - center[1], start[0] - center[0])
        theta2 = np.arctan2(goal[1] - center[1], goal[0] - center[0])
        theta2 += 2*np.pi * in_q2(theta1)*in_q3(theta2)
        theta2 -= 2*np.pi * in_q2(theta2)*in_q3(theta1)
        n = abs(int((theta2 - theta1) / self.config['dtheta']))
        
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

        if not self.gcode_is_parsed():
            self.LOG_WARNING(f"move_parsed no movements stored")
            return False
        
        # Move to 100, 0, 25 as a good starting spot
        res = robot.move_xyz(100,0,25)
        if not res:
            self.LOG_ERROR("Failed to move according to g-code")
            return False

        # Set the velocity to something nice and slow
        robot.set_tcp_vel(self.config['tcp_vel'])

        for pos in self.pos_to_go:
            res = robot.moveL_xyz(*pos)             
            if not res:
                self.LOG_ERROR("Failed to move according to g-code")
                return False

                
        
        # Move to 100, 0, 25 as a good ending spot
        res = robot.move_xyz(100,0,25)
        self.LOG_INFO(f"Added all g-code movements")

        # Now gcode file has been used so set the current file to None
        self.reset()

        return True






g_code = GCode()
































