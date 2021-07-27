from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from robot.robot import robot
import numpy as np
from PIL import Image, ImageDraw
from misc import print_enable_disable

class HeatMap:
    """
        Class that generates a heatmap of possible x,y positions for the robot. It considers 
        the arm lengths of the robot and creates a square with sides 2*(L1+L2). Then all of 
        those x,y coordinates are run through the robots inverse kinematics function and then 
        the verify movement_data of the robot. All x,y pairs which are valid are then plotted as 
        white while the invalid are plotted as black. In the end it generates a PIL image
    """
    def __init__(self):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        
        # Read all the configs
        self.load_configs()

        # Params used for calculations
        self.x_max = robot.config['L1'] + robot.config['L2']
        self.x_min = -self.x_max
        self.y_max = self.x_max
        self.y_min = -self.x_max
        self.dx = self.config['dx']
        self.width = self.config['img_width']
        self.height = self.config['img_height']

        # These are conversions factors. They are calculated via update_conversions
        self.mm_to_px_x = None # float
        self.mm_to_px_y = None # float
        self.px_to_mm_x = None # float
        self.px_to_mm_y = None # float
        self.update_conversions()

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"Inited HeatMap.\nConfig: {self.config},\nand base config: {self.config_base}")

    def update_conversions(self):
        self.mm_to_px_x = self.width / (self.x_max - self.x_min) 
        self.mm_to_px_y = self.height / (self.y_max - self.y_min) 
        self.px_to_mm_x = 1/self.mm_to_px_x
        self.px_to_mm_y = 1/self.mm_to_px_y

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

    def add_heatmap(self, n):
        """Adds the heatmap to self.heatmap

        Args:
            n ([type]): [description]
        """
        # Base values to put into validate movement data. Not important just need to be valid
        vel = robot.config['v_max']
        acc = robot.config['a_max']
        z = robot.config['z_max']
        gripper = robot.config['gripper_max']
        print_enable_disable.print_disable()

        for x_i, x in enumerate(np.linspace(self.x_min, self.x_max, n)):
            for y_i, y in enumerate(np.linspace(self.y_min, self.y_max, n)):
                J1, J2, J3 = robot.inverse_kinematics(x,y)
                if J1 is None:
                    continue

                # Figure out if any of the solutions are allowed
                any_allowed = None
                for i in range(2):
                    if J1[i] is None or J2[i] is None:
                        continue
                    if robot.validate_movement_data(J1[i], J2[i], J3, z, gripper, vel, acc):
                        any_allowed = True
                        break

                if any_allowed:
                    self.heatmap.putpixel((x_i,y_i), (255,255,255,255))
            
            # Print progress
            if self.verbose_level <= VerboseLevel.DEBUG:
                print_enable_disable.print_enable()
                print(f"{self.name} Done with {100*x_i/n:.3f} % of the image")
                print_enable_disable.print_disable()

        print_enable_disable.print_enable()

    def pixels_to_pos(self, x, y):
        """Takes the x and y position of pixel and returns the mm position it corresponds to

        Args:
            x (int): the x position of the pixel
            y (int): the y postion of the pixel
        """
        # Overlap the coordinate frames
        x -= self.width/2
        y -= self.height/2

        # Rotate around x axis
        y *= -1

        # Convert px to mm
        x = x*self.px_to_mm_x
        y = y*self.px_to_mm_y

        return x, y

    def pos_to_pixels(self, x, y):
        """Takes the a x, y coordinate in mm and returns the x and y location of the pixel it corresponds to in the heatmap.

        Args:
            x (int): the x position in mm
            y (int): the y postion in mm
        """
        # Overlap the coordinate frames
        x += self.width/2
        y += self.height/2

        # Rotate around x axis
        y *= -1

        # Convert pos to px
        x = x*self.mm_to_px_x
        y = y*self.mm_to_px_y

        x = int(x)
        y = int(y)

        return x, y

    def set_heatmap(self, heatmap:Image):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} Setting heatmap to {heatmap}")
        self.heatmap = heatmap
        self.width, self.height = heatmap.size()
        self.update_conversions()

    def generate_heatmap(self, width=None, height=None, save=True, path=None):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} Going to generate heatmap")

        n = np.ceil((self.x_max - self.x_min)/self.dx).astype(int)
        self.heatmap = Image.new('RGB', (n,n), color = (0, 0, 0))
        self.add_heatmap(n)
        
        # Reshape the img
        if width is not None:
            self.width = width 
            self.height = height

        self.heatmap = self.heatmap.resize((self.width, self.height))

        # Add robot ellipse
        draw = ImageDraw.Draw(self.heatmap)
        robot_radius = robot.config['base_radius']*self.mm_to_px_x
        draw.ellipse([(self.width//2-robot_radius,self.height//2-robot_radius),(self.width//2+robot_radius,self.height//2+robot_radius)], fill=(255,119,0))
        

        if save:
            if path is None:
                path = f"./imgs/{self.config['img_name']}.png"
            self.heatmap.save(path)
            if self.verbose_level <= VerboseLevel.DEBUG:
                print(f"{self.name} Saved heatmap as: {path}")

        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name} Done generating heatmap")

        return self.heatmap

    def show_heatmap(self, heatmap=None):
        if self.verbose_level <= VerboseLevel.DEBUG:
            print(f"{self.name}: Going to display heatmap")

        if heatmap is not None:
            heatmap.show()
        else:
            self.heatmap.show()

heatmap = HeatMap()
