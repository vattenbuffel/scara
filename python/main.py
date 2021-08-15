if __name__ == '__main__':
    from cli.cli import cli
    from robot.robot import robot
    from serial_data_communicator.serial_communicator import serial_com
    import streamlit as st
    from heatmap.heatmap import heatmap
    from g_code.g_code import g_code
    from simulator.simulator import simulator

    if st._is_running_with_streamlit:
        from gui.gui import loop
        loop()

    simulator.plot_start()
    simulator.home()
    simulator.move_xy(200, 0)

    print(f"Done in main\n{cli.prompt}", end="")