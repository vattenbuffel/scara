import streamlit as st
from robot.robot import robot
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os


if 'updated_pos' not in st.session_state:
    st.session_state.updated_pos = False
if 'updated_joints' not in st.session_state:
	st.session_state.updated_joints = False
if 'updated_gripper' not in st.session_state:
	st.session_state.updated_gripper = False
if 'updated_home' not in st.session_state:
	st.session_state.updated_home = False
if 'config' not in st.session_state:
    fp = Path(__file__)
    config_fp = os.path.join(str(fp.parent), "config.yaml")
    with open(config_fp) as f:
        st.session_state.config = yaml.load(f, Loader=yaml.FullLoader)
    config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
    with open(config_fp) as f:
        st.session_state.config_base = yaml.load(f, Loader=yaml.FullLoader)

    st.session_state.verbose_level = VerboseLevel.str_to_level(st.session_state.config_base['verbose_level'])
    st.session_state.name = st.session_state.config['name']

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
            if st.session_state.verbose_level <= VerboseLevel.DEBUG:
                print(f"{st.session_state.name}: Moving to pos x:{z_slider}, y:{y_slider}, z:{z_slider}") 
            robot.goto_pos(x_slider, y_slider, z_slider)
            st.session_state['updated_pos'] = False

    with col2:
        st.subheader('Forward kinematics')
        j1_slider = st.slider("J1", -90, 90, 0, 1, on_change=lambda: set_update('updated_joints'))
        j2_slider = st.slider("J2", -90, 90, 0, 1, on_change=lambda: set_update('updated_joints'))
        j3_slider = st.slider("J3", -90, 90, 0, 1, on_change=lambda: set_update('updated_joints'))
        if st.session_state['updated_joints']:
            if st.session_state.verbose_level <= VerboseLevel.DEBUG:
                print(f"{st.session_state.name}: Moving robot to J1:{j1_slider}, J2:{j2_slider}, J3:{j3_slider}, z:{z_slider}") 
            robot.goto_joints(j1_slider, j2_slider, j3_slider, in_rad=False)
            st.session_state['updated_joints'] = False

    st.subheader('Gripper')
    gripper_slider = st.slider("gripper", 0, 40, 0, 1, on_change=lambda: set_update('updated_gripper'))
    if st.session_state['updated_gripper']:
        if st.session_state.verbose_level <= VerboseLevel.DEBUG:
                print(f"{st.session_state.name}: updated_gripper to: {gripper_slider}")
        robot.alter_gripper(gripper_slider)
        st.session_state['updated_gripper'] = False

    st.button("Home", on_click=lambda: set_update('updated_home'))
    if st.session_state['updated_home']:
        if st.session_state.verbose_level <= VerboseLevel.DEBUG:
                print(f"{st.session_state.name}: Going home") 
        robot.home()
        st.session_state['updated_home'] = False


