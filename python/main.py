from cli.cli import cli
from robot import robot
from serial_data_communicator.serial_communicator import serial_com
from gui.gui import loop


if __name__ == '__main__':
    loop()
    print("Done in main")
    