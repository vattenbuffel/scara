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
        Class used for simulating 2D movements. It ignores the z-axis all together and only bothers with x and y.
        As a result only J1 and J2 are interesting. It inherits robot and overrides _home and _move to not send 
        data via queue_sender to the arduino but instead plots them. The main use for this is to verify that the
        path planner works so that that tcp movements are linear. 
    """
    def __init__(self):
        self.config = None # dict
        self.config_base = None # dict
        self.name = None # str
        self.verbose_level = None # misc.verbosity_level

        self.J1_ravg_vel = RollingAverage(10)
        self.J2_ravg_vel = RollingAverage(10)
        self.tcp_ravg_vel = RollingAverage(10)
        
        # Init the robot
        super().__init__()
        # Read all the configs
        self.load_configs()

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

    def load_configs(self):
        # Load super configs well
        super().load_configs()

        fp = Path(__file__)
        config_fp = os.path.join(str(fp.parent), "config.yaml")
        with open(config_fp) as f:
            self.config.update(yaml.load(f, Loader=yaml.FullLoader))
        config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
        with open(config_fp) as f:
            self.config_base.update(yaml.load(f, Loader=yaml.FullLoader))

        self.name = self.config['name']
        self.verbose_level = VerboseLevel.str_to_level(self.config_base['verbose_level'])

    def _home(self, *arg):
        """Overwrites robot's _home function to make it just set cur_pos J1,J2,J3,z = 0,0,0,0
        """
        # self.LOG_DEBUG(f"Going home.")
        
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
        """Add the correct data to the pos_queue and vel_queue used for plotting
        """
        x1 = self.config['L1']*np.cos(self.J1)
        y1 = self.config['L1']*np.sin(self.J1)
        x2 = self.x
        y2 = self.y

        J1_vel = self.J1_ravg_vel.get_avg()
        J2_vel = self.J2_ravg_vel.get_avg()
        tcp_vel_mms = self.tcp_ravg_vel.get_avg()

        if overwrite:
            try:
                self.pos_queue.get_nowait()
                self.vel_queue.get_nowait()
            except Empty:
                pass

        self.pos_queue.put([(x1, y1), (x2, y2)])
        self.vel_queue.put((J1_vel, J2_vel, tcp_vel_mms))

    def _move_robot(self, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy):
        """Simulates the movements of the robot. Ignores accuracy and acceleration

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
        # self.LOG_DEBUG(f"Going to move robot to J1: {J1}, J2: {J2}, J3: {J3}, z:{z}, gripper_value:{gripper_value}")

        # Make sure the pos wanted is possible
        success = self.validate_movement_data(J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc)
        if not success:
            self.LOG_WARNING(f"Failed with move robot")
            return

        now_ns = 0 

        J1_dt_ns = 1e9/self.deg_to_steps_J1(J1_vel, in_rad=False) / self.config['speed_factor'] 
        J1_prev_ns = now_ns
        J1_epsilon = self.steps_to_deg_J1(1)
        J1_vel_sign = 1 if J1 > self.J1 else -1
        J1_done = self.J_done(self.J1, J1, J1_epsilon)
        J1_end_time_ns = 0
        J1_start = self.J1
        J1_sim_steps = np.floor(abs(J1-J1_start)/J1_epsilon) # How many steps J1 motor will have to take

        J2_dt_ns = 1e9/self.deg_to_steps_J2(J2_vel, in_rad=False) / self.config['speed_factor'] 
        J2_prev_ns = now_ns
        J2_epsilon = self.steps_to_deg_J2(1)
        J2_vel_sign = 1 if J2 > self.J2 else -1
        J2_done = self.J_done(self.J2, J2, J2_epsilon)
        J2_end_time_ns = 0
        J2_start = self.J2
        J2_sim_steps = np.floor(abs(J2-J2_start)/J2_epsilon) # How many steps J2 motor will have to take

        tcp_start = self.x, self.y
        x_prev, y_prev = tcp_start
        tcp_prev_ns = now_ns

        sim_start_ns = now_ns

        while not self.kill_event.is_set():
            if J1_done and J2_done:
                break

            # J1
            if J1_done:
                now_ns += min(J2_dt_ns, J2_prev_ns + J2_dt_ns - now_ns) 
            elif J2_done:
                now_ns += min(J1_dt_ns, J1_prev_ns + J1_dt_ns - now_ns) 
            else:
                now_ns += min(J1_prev_ns + J1_dt_ns - now_ns, J2_prev_ns + J2_dt_ns - now_ns)

            if  now_ns  >=  J1_prev_ns + J1_dt_ns and not J1_done:
                J1_vel = self.steps_to_deg_J1(1/((now_ns - J1_prev_ns)/1e9), in_rad=False)*J1_vel_sign
                self.J1_ravg_vel.update(J1_vel) # Update the average velocity of J1
                self.J1 += self.steps_to_deg_J1(1)*J1_vel_sign
                J1_prev_ns = now_ns 

                J1_done = self.J_done(self.J1, J1, J1_epsilon)
                if J1_done: 
                    J1_end_time_ns = now_ns

            # J2
            if  now_ns  >=  J2_prev_ns + J2_dt_ns and not J2_done:
                J2_vel = self.steps_to_deg_J2(1/((now_ns - J2_prev_ns)/1e9), in_rad=False) * J2_vel_sign
                self.J2_ravg_vel.update(J2_vel) # Update the average velocity of J2
                self.J2 += self.steps_to_deg_J2(1)*J2_vel_sign
                J2_prev_ns = now_ns 

                J2_done = self.J_done(self.J2, J2, J2_epsilon)
                if J2_done: 
                    J2_end_time_ns = now_ns

            # Update position
            self.x, self.y = self.forward_kinematics(self.J1, self.J2)

            # tcp
            if now_ns >= tcp_prev_ns + max(J1_dt_ns, J2_dt_ns):
                d = ((x_prev-self.x)**2 + (y_prev-self.y)**2)**0.5
                tcp_vel_mms = d / ((now_ns - tcp_prev_ns) / 1e9)
                self.tcp_ravg_vel.update(tcp_vel_mms)
                x_prev, y_prev = self.x, self.y
                tcp_prev_ns = now_ns

            # Don't plot if there's too little data
            if self.J1_ravg_vel.n == len(self.J1_ravg_vel.vals):
                self.feed_plot()


        sim_end_ns = now_ns
        sim_time = (sim_end_ns - sim_start_ns)/1e9

        J1_time = (J1_end_time_ns - sim_start_ns)/1e9
        J1_vel_step = self.deg_to_steps_J1(J1_start - J1) / J1_time if J1_time else 0
        J1_vel_deg = np.rad2deg(J1_start - J1) / J1_time if J1_time else 0

        J2_time = (J2_end_time_ns - sim_start_ns)/1e9
        J2_vel_step = self.deg_to_steps_J1(J2_start - J2) / J2_time if J2_time else 0
        J2_vel_deg = np.rad2deg(J2_start - J2) / J2_time if J2_time else 0

        tcp_vel_mms = ((self.x - tcp_start[0])**2 + (self.y - tcp_start[1])**2)**0.5 / sim_time
        
        self.LOG_INFO(f"It took: {sim_time:.2f} s to complete simulation with speed_factor: {self.config['speed_factor']:.2f}")
        self.LOG_INFO(f"That corresponds to J1_vel: {J1_vel_step:.2f} steps/s = {J1_vel_deg:.2f} deg/s and J2_vel: {J2_vel_step:.2f} steps/s = {J2_vel_deg:.2f} deg/s and tcp_vel: {tcp_vel_mms:.2f} mm/s")
        self.LOG_INFO(f"It took J1 {J1_sim_steps} steps and J2: {J2_sim_steps} steps")
        # self.LOG_DEBUG(f"At pose J1: {J1}, J2: {J2}")


        J1_expected_sim_time_ns = J1_dt_ns *  J1_sim_steps
        assert abs(J1_end_time_ns - J1_expected_sim_time_ns) < 1e-3 or J1_time == 0
        J2_expected_sim_time_ns = J2_dt_ns * J2_sim_steps
        assert abs(J2_end_time_ns - J2_expected_sim_time_ns) < 1e-3 or J2_time == 0

    def J_done(self, J, goal, eps):
        return eps >= abs(goal - J) 

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