from simulator.plot_params import PlotParam
from collections import deque
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from queue import Empty
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

class PauseAnimation:
    def __init__(self, plot_params:PlotParam):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level
        self.plot_params = plot_params
        
        # Read all the configs
        self.load_configs()

        fig, (self.ax_robot, self.ax_vel) = plt.subplots(1,2)
        self.ax_robot.set_ylim(plot_params.y_min, plot_params.y_max)
        self.ax_robot.set_xlim(plot_params.x_min, plot_params.x_max)
        self.ax_robot.set_title('Robot')
        self.plot_robot, = self.ax_robot.plot(0, 0, label="Robot")
        self.plot_tcp_past, = self.ax_robot.plot(0,0, label="Past tcp")
        self.ax_robot.legend(loc=2)

        self.ax_vel.set_ylim(plot_params.vel_min, plot_params.vel_max)
        self.ax_vel.set_xlim(0, self.config['vel_plot_n']-1)
        self.ax_vel.set_title('Velocity')
        self.plot_J1_vel, = self.ax_vel.plot([0]*self.config['vel_plot_n'], label="J1")
        self.plot_J2_vel, = self.ax_vel.plot([0]*self.config['vel_plot_n'], label="J2")
        self.plot_tcp_vel, = self.ax_vel.plot([0]*self.config['vel_plot_n'], label="Tcp")
        self.ax_vel.legend(loc=2)

        self.animation = FuncAnimation(fig, self.plot_fn, interval=0)
        self.paused = False

        self.vel_J1_deque = deque([0]*self.config['vel_plot_n'], maxlen=self.config['vel_plot_n'])
        self.vel_J2_deque = deque([0]*self.config['vel_plot_n'], maxlen=self.config['vel_plot_n'])
        self.vel_tcp_deque = deque([0]*self.config['vel_plot_n'], maxlen=self.config['vel_plot_n'])
        self.x_tcp_deque = deque(maxlen=self.config['robot_plot_past_pos_n'])
        self.y_tcp_deque = deque(maxlen=self.config['robot_plot_past_pos_n'])

        fig.canvas.mpl_connect('button_press_event', self.toggle_pause)

        plt.show()

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

    def toggle_pause(self, *args, **kwargs):
        if self.paused:
            self.animation.resume()
        else:
            self.animation.pause()
        self.paused = not self.paused
        
        print("Pausing" if self.paused else "Starting")

    def plot_fn(self, i):
        try:
            # Update robot plot
            data = self.plot_params.pos_queue.get_nowait()
            x1, y1, x2, y2 = data[0][0], data[0][1], data[1][0], data[1][1]
            self.plot_robot.set_xdata([0, x1, x2])
            self.plot_robot.set_ydata([0, y1, y2])
            self.x_tcp_deque.append(x2)
            self.y_tcp_deque.append(y2)
            self.plot_tcp_past.set_xdata(self.x_tcp_deque)
            self.plot_tcp_past.set_ydata(self.y_tcp_deque)


            # Update vel plot
            J1_vel, J2_vel, tcp_vel = self.plot_params.vel_queue.get() # This should not be able to be empty since pos_queue wasn't empty
            self.vel_J1_deque.append(J1_vel)
            self.vel_J2_deque.append(J2_vel)
            self.vel_tcp_deque.append(tcp_vel)
            self.plot_J1_vel.set_ydata(self.vel_J1_deque)
            self.plot_J2_vel.set_ydata(self.vel_J2_deque)
            self.plot_tcp_vel.set_ydata(self.vel_tcp_deque)


        except Empty:
            pass

        return (self.plot_robot, self.plot_J1_vel, self.plot_J2_vel, self.plot_tcp_vel)

def plot(plot_params:PlotParam):
    while True:
        plot_params.start_event.wait()
        plot_params.start_event.clear()
        pause_animation = PauseAnimation(plot_params)




