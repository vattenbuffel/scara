import multiprocessing
from queue import Empty
from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

class PauseAnimation:
    def __init__(self, pos_queue, x_min, y_min, x_max, y_max):
        fig, self.ax = plt.subplots()
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_xlim(x_min, x_max)
        self.ax.set_title('Simulation')
        self.p, = self.ax.plot(0, 0)

        self.animation = FuncAnimation(fig, self.plot_fn, interval=0)
        self.paused = False

        self.pos_queue:multiprocessing.Queue = pos_queue # Should be populated by simulator module. Will contain the pos of J1 end and J2 end: [(x1,y1), (x2,y2)]

        fig.canvas.mpl_connect('button_press_event', self.toggle_pause)

        plt.show()

    def toggle_pause(self, *args, **kwargs):
        if self.paused:
            self.animation.resume()
        else:
            self.animation.pause()
        self.paused = not self.paused
        
        print("Pausing" if self.paused else "Starting")

    def plot_fn(self, i):
        try:
            data = self.pos_queue.get_nowait()
            x1, y1, x2, y2 = data[0][0], data[0][1], data[1][0], data[1][1]
            self.p.set_xdata([0, x1, x2])
            self.p.set_ydata([0, y1, y2])
        except Empty:
            pass

        return (self.p, )

def plot(start_event, pos_queue:multiprocessing.Queue, x_min, y_min, x_max, y_max):
    while True:
        start_event.wait()
        start_event.clear()
        pause_animation = PauseAnimation(pos_queue, x_min, y_min, x_max, y_max)



    # while pause_event.wait():
    #     if kill_event.is_set():
    #         break

