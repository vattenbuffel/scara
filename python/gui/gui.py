import streamlit as st
from robot.robot import robot
from message.message_types import MessageTypes
import threading
from serial_data_communicator.handy_functions import handy_functions
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
from message.message_updated import MessageUpdated


if 'updated_pos' not in st.session_state:
    st.session_state.updated_pos = False
if 'updated_joints' not in st.session_state:
	st.session_state.updated_joints = False
if 'updated_gripper' not in st.session_state:
	st.session_state.updated_gripper = False
if 'updated_home' not in st.session_state:
	st.session_state.updated_home = False

def set_update(key):
    st.session_state[key]=True

def loop():
    col1, col2 = st.beta_columns(2)

    with col1:
        st.subheader('Inverse kinematics')
        x_slider = st.slider("x", 0, 360, 1, 1, on_change=lambda: set_update('updated_pos'))
        y_slider = st.slider("y", 0, 360, 1, 1, on_change=lambda: set_update('updated_pos'))
        z_slider = st.slider("z", 0, 360, 1, 1, on_change=lambda: set_update('updated_pos'))
        if st.session_state['updated_pos']:
            print(f"Moving to pos x:{z_slider}, y:{y_slider}, z:{z_slider}") #TODO Make it consider verbosity levels
            robot.goto_pos(x_slider, y_slider, z_slider)
            st.session_state['updated_pos'] = False

    with col2:
        st.subheader('Forward kinematics')
        j1_slider = st.slider("J1", 0, 360, 1, 1, on_change=lambda: set_update('updated_joints'))
        j2_slider = st.slider("J2", 0, 360, 1, 1, on_change=lambda: set_update('updated_joints'))
        j3_slider = st.slider("J3", 0, 360, 1, 1, on_change=lambda: set_update('updated_joints'))
        if st.session_state['updated_joints']:
            print(f"Moving to pos J1:{j1_slider}, J2:{j2_slider}, J3:{j3_slider}") #TODO Make it consider verbosity levels
            robot.goto_joints(j1_slider, j2_slider, j3_slider)
            st.session_state['updated_joints'] = False

    st.subheader('Gripper')
    gripper_slider = st.slider("gripper", 0, 100, 0, 1, on_change=lambda: set_update('updated_gripper'))
    if st.session_state['updated_gripper']:
        print(f"updated_gripper to: {gripper_slider}") #TODO Make it consider verbosity levels
        robot.alter_gripper(gripper_slider)
        st.session_state['updated_gripper'] = False

    st.button("Home", on_click=lambda: set_update('updated_home'))
    if st.session_state['updated_home']:
        print(f"Going home") #TODO Make it consider verbosity levels
        robot.home()
        st.session_state['updated_home'] = False


