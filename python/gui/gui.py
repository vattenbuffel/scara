import streamlit as st
from robot.robot import robot
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os

class MovementData:
    def __init__(self, should_move, val):
        self.should_move = should_move
        self.val = val
        

if 'init' not in st.session_state:
    st.session_state.init = False

    st.session_state.update_fns = {'update_x':robot.move_x, 'update_y':robot.move_y, 'update_z':robot.move_z, 'update_J1':robot.move_J1, 'update_J2':robot.move_J2, 'update_J3':robot.move_J3, 'update_gripper':robot.alter_gripper, 'update_home':lambda *data: robot.home()}
    for key in st.session_state.update_fns:
        if key not in st.session_state:
            st.session_state[key] = MovementData(False, 0)

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
    st.session_state[key].should_move = True



def loop():
    col1, col2 = st.beta_columns(2)

    with col1:
        st.subheader('Inverse kinematics')
        st.session_state['update_x'].val = st.slider("x", 0, 360, 1, 1, on_change=lambda: set_update('update_x'))
        st.session_state['update_y'].val = st.slider("y", 0, 360, 1, 1, on_change=lambda: set_update('update_y'))
        st.session_state['update_z'].val = st.slider("z", 0, 360, 1, 1, on_change=lambda: set_update('update_z'))
        
        print(f"After col1: {st.session_state['update_x']}")

    with col2:
        st.subheader('Forward kinematics')
        st.session_state['update_J1'].val = st.slider("J1", -90, 90, 0, 1, on_change=lambda: set_update('update_J1'))
        st.session_state['update_J2'].val = st.slider("J2", -90, 90, 0, 1, on_change=lambda: set_update('update_J2'))
        st.session_state['update_J3'].val = st.slider("J3", -90, 90, 0, 1, on_change=lambda: set_update('update_J3'))

    st.subheader('Gripper')
    st.session_state['update_gripper'].val = st.slider("gripper", 0, 40, 0, 1, on_change=lambda: set_update('update_gripper'))

    # Home button doesn't need a value
    st.button("Home", on_click=lambda: set_update('update_home'))

    for key in st.session_state.update_fns:
        if st.session_state[key].should_move:
            if st.session_state.verbose_level <= VerboseLevel.DEBUG:
                print(f"{st.session_state.name}: Action: {key}, moving to {st.session_state[key].val}") 
            st.session_state.update_fns[key](st.session_state[key].val)

            # Since it's done moving should_move should be False
            st.session_state[key].should_move = False

