import time


if __name__ == '__main__':
    from cli.cli import cli
    from robot.robot import robot
    from serial_data_communicator.serial_communicator import serial_com
    import streamlit as st
    from heatmap.heatmap import heatmap
    from g_code.g_code import g_code
    from simulator.simulator import simulator
    from misc.kill import kill

    if st._is_running_with_streamlit:
        from gui.gui import loop
        loop()

    print(f"Done in main\n{cli.prompt}", end="")
    while not simulator.is_idle():
        time.sleep(1)