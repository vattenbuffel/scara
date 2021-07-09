import threading
from robot.robot_cmd_types import RobotCmdTypes

class RobotCmd:
    """
    RobotCmd is a helper class to robot. Whenever the robot should move, this class handles 
    the connection between the requesting thread and the robot thread. This will contain the
    neccessary data for the command to be executed. For example when calling move robot 
    the joint, z and grip values is contained. It also contains an event that will trigger 
    when the request has been done
    """
    def __init__(self):
        self.data = None
        self.type = RobotCmdTypes.NONE 
        self.event = threading.Event()
        