from robot.robot import robot
from message.message_types import MessageTypes
import threading
from serial_data_communicator.handy_functions import handy_functions
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from message.message_updated import MessageUpdated

import streamlit as st


# if 'updated' not in st.session_state:
# 	st.session_state.updated = False

# def set_update():
#     st.session_state.updated=True

def loop():
    col1, col2 = st.beta_columns(2)

    with col1:
        st.subheader('Inverse kinematics')
        x_slider = st.slider("x", 0, 360, 1, 1)
        y_slider = st.slider("y", 0, 360, 1, 1)
        z_slider = st.slider("z", 0, 360, 1, 1)
        # x_slider = st.slider("x", 0, 360, 1, 1, on_change=set_update)
        # y_slider = st.slider("y", 0, 360, 1, 1, on_change=set_update)
        # z_slider = st.slider("z", 0, 360, 1, 1, on_change=set_update)
        print(f"Moving to pos x:{z_slider}, x:{y_slider}, x:{z_slider}")
        robot.goto_pos(x_slider, y_slider, z_slider)

    with col2:
        st.subheader('Forward kinematics')
        j1_slider = st.slider("J1", 0, 360, 1, 1)
        j2_slider = st.slider("J2", 0, 360, 1, 1)
        j3_slider = st.slider("J3", 0, 360, 1, 1)

    st.subheader('Gripper')
    gripper_slider = st.slider("gripper", 0, 100, 0, 1)

    st.button("Home", on_click=lambda: print("Gonna home"))


