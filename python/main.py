from cli.cli import cli
from robot.robot import robot
from serial_data_communicator.serial_communicator import serial_com
import streamlit as st
from heatmap.heatmap import heatmap
from g_code.g_code import g_code

if st._is_running_with_streamlit:
    from gui.gui import loop


if __name__ == '__main__':
    if st._is_running_with_streamlit:
        loop()
    g_code.parse()
    print(f"Done in main\n{cli.prompt}", end="")