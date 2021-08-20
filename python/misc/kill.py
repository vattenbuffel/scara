import sys
from robot.robot import robot
from serial_data_communicator.serial_communicator import serial_com
from serial_data_communicator.handy_functions import handy_functions
from simulator.simulator import simulator
from queue_sender.queue_sender import queue_sender 



def kill():
   """Function that kills all modules in the program and callinc sys.exit, 
   effectively killing everything and stopping the program.
   """
   print("Kill program")
   # Avoid import errors. This signals that the cli tries to stop the program. In this case cli is responsible for
   # stopping itself.
   try:
      from cli.cli import cli
   except ImportError:
      cli = False

   robot.kill()
   serial_com.kill()
   # gui.kill()
   handy_functions.kill()
   simulator.kill()
   queue_sender.kill()
   if cli:
      cli.kill()

   sys.exit()