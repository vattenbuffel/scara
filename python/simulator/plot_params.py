from multiprocessing import Event, Queue

class PlotParam:
    """This is a class containing all the data needed to be passed from the simulator module to the plotting process.
    """

    def __init__(self, start_event:Event, pos_queue:Queue, x_min, y_min, x_max, y_max, vel_queue:Queue, vel_min, vel_max):
        self.start_event = start_event
        self.pos_queue:Queue = pos_queue # Should be populated by simulator module. Will contain the pos of J1 end and J2 end: [(x1,y1), (x2,y2)]
        self.vel_queue:Queue = vel_queue # Populated by simulator module. [v_J1, v_J2, v_tcp]
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.vel_queue = vel_queue
        self.vel_min = vel_min
        self.vel_max = vel_max