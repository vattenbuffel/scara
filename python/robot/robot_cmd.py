from robot.robot_cmd_types import RobotCmdTypes

class RobotCmd:
    """
    RobotCmd is a helper class to robot. Whenever the robot should move, this class handles 
    the connection between the requesting thread and the robot thread. This will contain the
    neccessary data for the command to be executed. For example when calling move robot 
    the joint, z and grip values is contained. It also contains an event that will trigger 
    when the request has been done
    """
    def __init__(self, type_:RobotCmdTypes, J1, J2, J3, z, gripper_value, J1_vel, J2_vel, J3_vel, z_vel, J1_acc, J2_acc, J3_acc, z_acc, accuracy):
        self.type = type_
        self.J1 = J1
        self.J2 = J2
        self.J3 = J3
        self.z = z
        self.gripper_value = gripper_value
        self.J1_vel = J1_vel
        self.J2_vel = J2_vel
        self.J3_vel = J3_vel
        self.z_vel = z_vel
        self.J1_acc = J1_acc
        self.J2_acc = J2_acc
        self.J3_acc = J3_acc
        self.z_acc = z_acc
        self.accuracy = accuracy

    def data(self):
        return (self.J1, self.J2, self.J3, self.z, self.gripper_value, self.J1_vel, self.J2_vel, self.J3_vel, self.z_vel, self.J1_acc, self.J2_acc, self.J3_acc, self.z_acc, self.accuracy)

    def __str__(self):
        return f"Type: {self.type}, data: {self.data()}"
        