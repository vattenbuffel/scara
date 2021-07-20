from cli.cli import cli
from robot.robot import robot
from serial_data_communicator.serial_communicator import serial_com
import streamlit as st

if st._is_running_with_streamlit:
    from gui.gui import loop


if __name__ == '__main__':
    if st._is_running_with_streamlit:
        loop()
    print("Done in main")