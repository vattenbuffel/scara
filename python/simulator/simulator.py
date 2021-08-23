from queue import Empty
from misc.rolling_average import RollingAverage
import time
import numpy as np
import yaml
from pathlib import Path
import os
from robot.robot import Robot
import multiprocessing
from simulator.plot_params import PlotParam
from simulator import plot_process
from misc.verbosity_levels import VerboseLevel


class Simulator(Robot):
    """
        Class used for simulating 2D movements. It ignores the z-axis all together and only bother with x and y.
        As a result only J1 and J2 are interesting. It inherits robot and overrides _home and _move to not send 
        data via queue_sender to the arduino but instead plots them. The main use for this is to ensure that 
        tcp movements are linear. 
    """
    def __init__(self):
        self.sim_config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level

        # Read all the configs
        self.sim_load_configs()

        self.J1_ravg_vel = RollingAverage(10)
        self.J2_ravg_vel = RollingAverage(10)
        self.tcp_ravg_vel = RollingAverage(10)
        
        # Init the robot
        super().__init__()

        self.start_event = multiprocessing.Event() 
        self.pos_queue = multiprocessing.Queue()
        self.vel_queue = multiprocessing.Queue()
        size = (self.config['L1'] + self.config['L2'])*1.1
        v_min = -self.config['v_max']*1.1
        v_max = self.config['v_max']*1.1
        plot_params = PlotParam(self.start_event, self.pos_queue, -size, -size, size, size, self.vel_queue, v_min, v_max)
        self.plot_process = multiprocessing.Process(name=self.config['name']+"_process", target=plot_process.plot, args=(plot_params,))
        self.plot_process.start()



        self.LOG_INFO(f"Inited Simulator.\nConfig: {self.config},\nand base config: {self.config_base}")

    def sim_load_configs(self):
        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.sim_config = yaml.load(f, Loader=yaml.FullLoader)
        config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
        with open(config_fp) as f:
            self.config_base = yaml.load(f, Loader=yaml.FullLoader)

        self.name = self.sim_config['name']
        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])

    def _home(self, *arg):
        """Overwrites robot's _home function to make it just set cur_pos J1,J2,J3,z = 0,0,0,0
        """
        self.LOG_DEBUG(f"Going home.")
        
        x, y = self.forward_kinematics(0, 0)
        self.x = x 
        self.y = y 
        self.z = 0 
        self.J1 = 0 
        self.J2 = 0 
        self.J3 = 0 
        self.gripper_value = 0
        
        self.x_goal = x 
        self.y_goal = y 
        self.z_goal = 0 
        self.J1_goal = 0 
        self.J2_goal = 0 
        self.J3_goal = 0 
        self.gripper_value_goal = 0

        self.feed_plot(overwrite=False)

        self.LOG_INFO(f"At home, J1: {0}, J2: {0}, J3: {0}, x:{x}, y:{y}, z:{0}")

    def feed_plot(self, overwrite=True):
        """Add the correct data to the pos_queue used for plotting
        """
        x1 = self.config['L1']*np.cos(self.J1)
        y1 = self.config['L1']*np.sin(self.J1)
        x2 = x1 + self.config['L2']*np.cos(self.J1 + self.J2)
        y2 = y1 + self.config['L2']*np.sin(self.J1 + self.J2)

        J1_vel = self.J1_ravg_vel.get_avg()
        J2_vel = self.J2_ravg_vel.get_avg()
        tcp_vel = self.tcp_ravg_vel.get_avg()

        if overwrite:
            try:
                self.pos_queue.get_nowait()
                self.vel_queue.get_nowait()
            except Empty:
                pass

        self.pos_queue.put([(x1, y1), (x2, y2)])
        self.vel_queue.put((J1_vel, J2_vel, tcp_vel))

    def _move_robot(self, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy):
        """Simulates the movements of the robot. Ignores acc

        Args:
            J1 ([type]): [description]
            J2 ([type]): [description]
            J3 ([type]): [description]
            z ([type]): [description]
            gripper_value ([type]): [description]
            J1_vel ([type]): [description]
            J2_vel ([type]): [description]
            J3_vel ([type]): [description]
            z_vel ([type]): [description]
            J1_acc ([type]): [description]
            J2_acc ([type]): [description]
            J3_acc ([type]): [description]
            z_acc ([type]): [description]
            accuracy ([type]): [description]
        """
        self.LOG_DEBUG(f"Going to move robot to J1: {J1}, J2: {J2}, J3: {J3}, z:{z}, gripper_value:{gripper_value}")

        # Make sure the pos wanted are possible
        success = self.validate_movement_data(J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc)
        if not success:
            self.LOG_WARNING(f"Failed with move robot")
            return


        J1_dt_ns = 1e9/self.deg_to_steps_J1(J1_vel, in_rad=False) / self.sim_config['speed_factor'] 
        J1_prev_ns = time.perf_counter_ns()
        J1_epsilon = self.steps_to_deg_J1(1)
        J1_vel_sign = 1 if J1 > self.J1 else -1
        J1_offset_ns = 0 # Keeps track of how much timing error there are between steps and corrects it so that the resulting velocity is correct
        J1_done = False
        J1_vel = 0

        J2_dt_ns = 1e9/self.deg_to_steps_J2(J2_vel, in_rad=False) / self.sim_config['speed_factor'] 
        J2_prev_ns = time.perf_counter_ns()
        J2_epsilon = self.steps_to_deg_J2(1)
        J2_vel_sign = 1 if J2 > self.J2 else -1
        J2_offset_ns = 0
        J2_done = False
        J2_vel = 0

        sim_start_ns = time.perf_counter_ns()
        J1_end_time_ns = -1
        J2_end_time_ns = -1
        J1_start = self.J1
        J2_start = self.J2
        while not self.kill_event.is_set():
            if J1_done and J2_done:
                break

            # J1
            now_ns = time.perf_counter_ns()
            if  now_ns  >=  J1_prev_ns + J1_dt_ns + J1_offset_ns and not J1_done:
                J1_done = J1_epsilon >= np.abs(J1 - self.J1)
                if not J1_done:
                    J1_vel = self.steps_to_deg_J1(1/((now_ns - J1_prev_ns)/1e9), in_rad=False)*J1_vel_sign
                    self.J1_ravg_vel.update(J1_vel) # Update the average velocity of J1
                    self.J1 += self.steps_to_deg_J1(1)*J1_vel_sign
                    J1_offset_ns += J1_dt_ns - (now_ns - J1_prev_ns)
                    J1_prev_ns = now_ns 

                if J1_done: 
                    J1_end_time_ns = now_ns
                    J1_vel = 0

            # J2
            if  now_ns  >=  J2_prev_ns + J2_dt_ns + J2_offset_ns and not J2_done:
                J2_done = J2_epsilon >= np.abs(J2 - self.J2)
                if not J2_done:
                    J2_vel = self.steps_to_deg_J2(1/((now_ns - J2_prev_ns)/1e9), in_rad=False) * J2_vel_sign
                    self.J2_ravg_vel.update(J2_vel) # Update the average velocity of J2
                    self.J2 += self.steps_to_deg_J2(1)*J2_vel_sign
                    J2_offset_ns += J2_dt_ns - (now_ns - J2_prev_ns)
                    J2_prev_ns = now_ns 

                if J2_done: 
                    J2_end_time_ns = now_ns
                    J2_vel = 0

            # tcp
            tcp_vel = (J1_vel**2 + J2_vel**2)**0.5
            self.tcp_ravg_vel.update(tcp_vel)

            self.feed_plot()


        sim_end_ns = time.perf_counter_ns()
        sim_time = (sim_end_ns - sim_start_ns)/1e9

        J1_time = (J1_end_time_ns - sim_start_ns)/1e9
        J1_vel_step = self.deg_to_steps_J1(J1_start - J1) / J1_time
        J1_vel_deg = np.rad2deg(J1_start - J1) / J1_time

        J2_time = (J2_end_time_ns - sim_start_ns)/1e9
        J2_vel_step = self.deg_to_steps_J1(J2_start - J2) / J2_time
        J2_vel_deg = np.rad2deg(J2_start - J2) / J2_time
        
        self.LOG_DEBUG(f"It took: {sim_time:.2f} s to complete simulation with speed_factor: {self.sim_config['speed_factor']:.2f}")
        self.LOG_DEBUG(f"That corresponds to J1_vel: {J1_vel_step:.2f} steps/s = {J1_vel_deg:.5f} deg/s and J2_vel: {J2_vel_step:.2f} steps/s = {J2_vel_deg:.5f} deg/s")
        # Update position. 
        self.x, self.y = self.forward_kinematics(self.J1, self.J2)
        self.LOG_DEBUG(f"At pose J1: {J1}, J2: {J2}")

    def plot_start(self):
        self.LOG_DEBUG(f"Starting plotting")
        self.start_event.set()

    def kill(self):
        self.LOG_INFO(f"Dying")    

        self.LOG_DEBUG(f"Killing process: {self.plot_process.name}")
        self.plot_process.kill()

        super().kill()




 
if __name__ == 'simulator.simulator':
    print("Gonna create simulator")
    simulator = Simulator()