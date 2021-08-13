from io import StringIO
import time
import glob
from logger.logger import Logger
import streamlit as st
from robot.robot import robot
from misc.verbosity_levels import VerboseLevel
import yaml
from pathlib import Path
import os
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw
from streamlit_drawable_canvas import st_canvas
from heatmap.heatmap import heatmap
from g_code.g_code import g_code


st.set_page_config(page_title="test", layout="wide")
print("start of gui")

class MovementData:
    def __init__(self, should_move, val):
        self.should_move = should_move
        self.val = val
        
        

if 'init' not in st.session_state:
    st.session_state.init = False

    st.session_state.update_fns = { 'update_x':robot.move_x,
                                    'update_y':robot.move_y, 
                                    'update_z':robot.move_z,
                                    'update_J1':lambda data:robot.move_J1(data, in_rad=False),
                                    'update_J2':lambda data:robot.move_J2(data, in_rad=False), 
                                    'update_J3':lambda data:robot.move_J3(data, in_rad=False),
                                    'update_gripper':robot.alter_gripper, 
                                    'update_home':lambda *data: robot.home(),
                                    'update_vel': robot.set_velocity,
                                    'update_acc':robot.set_acceleration}

    for key in st.session_state.update_fns:
        if key not in st.session_state:
            st.session_state[key] = MovementData(False, 0)

if 'delete_file' not in st.session_state:
    st.session_state.delete_file = False
if 'upload_file_button' not in st.session_state:
    st.session_state.upload_file_button = False


if 'config' not in st.session_state:
    fp = Path(__file__)
    config_fp = os.path.join(str(fp.parent), "config.yaml")
    with open(config_fp) as f:
        st.session_state.config = yaml.load(f, Loader=yaml.FullLoader)
    config_fp = os.path.join(str(fp.parent.parent), "base_config.yaml")
    with open(config_fp) as f:
        st.session_state.config_base = yaml.load(f, Loader=yaml.FullLoader)
    
    
    # Load robot config so that min and max values for speed etc are known
    config_fp = os.path.join(str(fp.parent.parent), "./robot/config.yaml")
    with open(config_fp) as f:
        st.session_state.config_robot = yaml.load(f, Loader=yaml.FullLoader)

    st.session_state.verbose_level = VerboseLevel.str_to_level(st.session_state.config_base['verbose_level'])
    st.session_state.name = st.session_state.config['name']

if 'logger' not in st.session_state:
    st.session_state.logger = Logger(st.session_state.name, st.session_state.verbose_level)

if 'gcode_preview' not in st.session_state:
    st.session_state.gcode_preview = False
    st.session_state.gcode_preview_img = None

def set_update(key):
    st.session_state[key].should_move = True

def gcode_preview():
    if st.session_state.gcode_preview:
        st.markdown("## Preview")
        st.image(st.session_state.gcode_preview_img)

def set_gcode_preview(img):
    st.session_state.gcode_preview = True
    st.session_state.gcode_preview_img = img


def gcode_mode():
    col1, col2 = st.beta_columns(2)

    def get_paths():
        paths = glob.glob(f"{g_code.config['base_path']}*.gcode")
        # Sanitize paths
        paths_clean = [os.path.basename(paths[i]) for i in range(len(paths))]
        st.session_state.logger.LOG_DEBUG(f"g_code files found: {paths}")
        return paths, paths_clean
    paths, paths_clean = get_paths()

    with col1:
        st.header(F"Move according to g_code file")
        path_clean = st.selectbox(" ", paths_clean)
        st.session_state.logger.LOG_DEBUG(f"Selected file: {path_clean}")
        confirmed_choice = st.button("Move")
        st.write(f"chosen file: {path_clean}")
        if path_clean and confirmed_choice:
            st.session_state.logger.LOG_DEBUG(f"Going to move according to {path_clean}")

            for path in paths:
                if path_clean in path:
                    break

            g_code.set_gcode_file(path)
            g_code.parse()

            img = g_code.generate_img(scale=5)
            # Resize the image so that it's not too big for streamlit
            scale = heatmap.config['img_width']/max(img.size)
            img = img.resize((int(img.size[0]*scale), int(img.size[1]*scale)))
            # Add an outline to the img
            img1 = ImageDraw.Draw(img)
            end_points = tuple(np.array(img.size) - np.array([1,1]))
            img1.rectangle([(0,0), end_points], outline="grey")
            set_gcode_preview(img)
            gcode_preview()

            # Start the movements
            g_code.move_parsed()
            n_cmds = len(robot.get_cmds()[1])

            # Show a progress bar
            progress_bar = st.progress(0)
            progress_text = st.empty()
            while len(robot.get_cmds()[1]):
                progress = (n_cmds - len(robot.get_cmds()[1]))/n_cmds
                progress_text.text(f"Progress: {100*progress:.3f} %")
                progress_bar.progress(progress)
                time.sleep(0.1)

        
        gcode_preview()



    with col2:
        st.markdown("## Modify files")
        def upload_file_button_true():
            st.session_state.upload_file_button = True

        st.markdown("### Upload g_code")
        file = st.file_uploader("", type=".gcode")
        st.button("Upload", on_click=upload_file_button_true)
        # Copy contents of file into it's proper location
        if file is not None and st.session_state.upload_file_button == True:
            stringio = StringIO(file.getvalue().decode("utf-8"))

            # Make sure file is not duplicate
            if file.name in paths:
                st.warning(f"{file.name} already exists")
                st.session_state.logger.LOG_WARNING(f"Tried to upload file: {file.name} to dir: {g_code.config['base_path']}, but that file already exists there")
            else:
                st.session_state.logger.LOG_INFO(f"Going to upload file: {file.name} to dir: {g_code.config['base_path']}")
                f = open(f"{g_code.config['base_path']}{file.name}", "w")
                for line in file:
                    f.write(stringio.readline().strip('\n')) # Why loop over line when readline anyways? If it ain't broke, don't fix it
                f.close()
                st.session_state.logger.LOG_INFO(f"Successfully uploaded file: {file.name} to dir: {g_code.config['base_path']}")
                st.success("Uploaded file")

            st.session_state.upload_file_button = False

        # Delete a file
        st.markdown("### Delete a file")
        def delete_file_true():
            st.session_state.delete_file = True

        path_to_delete = st.selectbox("", paths)
        st.button("Delete", key="DELETE_GCODE_FILE_BUTTON", on_click=delete_file_true)
        if path_to_delete and st.session_state.delete_file:
            st.session_state.logger.LOG_DEBUG(f"Going to delete file: {path_to_delete}")
            os.remove(f"{g_code.config['base_path']}{path_to_delete}")
            st.session_state.logger.LOG_INFO(f"Successfully deleted file: {path_to_delete} from dir: {g_code.config['base_path']}")
            st.success("Deleted file")
            st.session_state.delete_file = False
                
    _, col2, _ = st.beta_columns(3)
    with col2:
        st.button("Update")






def extract_circle_centers(canvas_result):
    df = pd.json_normalize(canvas_result.json_data["objects"])
    centers = []

    if len(df) != 0:
        df["center_x"] = df["left"] + df["radius"] * np.cos(
            df["angle"] * np.pi / 180
        )
        df["center_y"] = df["top"] + df["radius"] * np.sin(
            df["angle"] * np.pi / 180
        )

        
        st.subheader("List of circle drawings")
        for _, row in df.iterrows():
            centers.append((row["center_x"], row["center_y"]))
    return centers


def heatmap_mode():
    col1, col2 = st.beta_columns(2)

    # Either load or create the heatmap
    img_name = f"./imgs/{heatmap.config['img_name']}.png"
    try:
        img = Image.open(img_name)
    except FileNotFoundError:
        st.session_state.logger.LOG_DEBUG(f"Couldn't find heat map with name: {img_name}, creating one")
        st.error("Couldn't find a heatmap. Creating it. This will take a while")
        img = heatmap.generate_heatmap()

    # Create a canvas component
    with col1:
        st.title("Possible positions")
        canvas_result = st_canvas(
            fill_color="rgba(255, 165, 0, 0.3)",  # Fixed fill color with some opacity
            stroke_width=1,
            stroke_color="#000000",
            background_color="#eee",
            background_image=img,
            update_streamlit=True,
            width=650, #Load these from the heatmap config
            height=650,
            drawing_mode="circle",
        )

    # Do something interesting with the image data and paths
    with col2:
        st.title("Goal positions")
        if canvas_result.image_data is not None:
            # Draw an image with all of the goal positions in order
            img = Image.new('RGB', (650,650), color = (255, 255, 255))
            draw = ImageDraw.Draw(img)
            
            # # Take the positions and add them to this image, but instead of circles, add them as numbers
            centers = extract_circle_centers(canvas_result)
            for i, center in enumerate(centers):
                draw.text(center, str(i), fill="black", align="center", stroke_width=10)

            st.image(img)

        # For debugging:
        # if canvas_result.json_data is not None:
        #     st.dataframe(pd.json_normalize(canvas_result.json_data["objects"]))

        send = st.button("Send")

        if send:
            for center in extract_circle_centers(canvas_result):
                st.session_state.logger.LOG_DEBUG(f"Moving robot to (x,y): {center}")
                x,y = heatmap.pixels_to_pos(*center)
                robot.move_xy(x, y)
                        

    # Display help text
    col1, col2, _ = st.beta_columns(3)
    with col2:
        '''
        # Help

        This is a visualisation tool to help move the robot. The left image is a heatmap of possible positions. 
        The columns corresponds to x and rows to y. Black means that the position cannot be reached and white 
        that it can be reached. Clicking on the left image creates a goal position of the robot. These can be 
        seen in the right image but without the interfering heatmap. Pressing send will move the robot into the 
        goal positions in the order they were placed.

        Note that no validation of the movements are performed before the send command is actually pressed. This 
        means that you can input incorrect positions without knowing.
        '''


def normal_mode():
    col1, col2 = st.beta_columns(2)

    with col1:
        st.subheader('Inverse kinematics')
        x_max = int(robot.config['L1'] + robot.config['L2'])
        st.session_state['update_x'].val = st.slider("x", -x_max, x_max, 1, 1, on_change=lambda: set_update('update_x'))
        st.session_state['update_y'].val = st.slider("y", -x_max, x_max, 1, 1, on_change=lambda: set_update('update_y'))
        st.session_state['update_z'].val = st.slider("z", st.session_state.config_robot['z_min'], st.session_state.config_robot['z_max'], 1, 1, on_change=lambda: set_update('update_z'))
        st.session_state['update_vel'].val = st.slider("velocity", st.session_state.config_robot['v_min'], st.session_state.config_robot['v_max'], st.session_state.config_robot['base_vel_J1'], 1, on_change=lambda: set_update('update_vel'))

    with col2:
        st.subheader('Forward kinematics')
        J1_min = int(np.rad2deg(st.session_state.config_robot['J1_min']))
        J1_max = int(np.rad2deg(st.session_state.config_robot['J1_max']))
        J2_min = int(np.rad2deg(st.session_state.config_robot['J2_min']))
        J2_max = int(np.rad2deg(st.session_state.config_robot['J2_max']))
        J3_min = int(np.rad2deg(st.session_state.config_robot['J3_min']))
        J3_max = int(np.rad2deg(st.session_state.config_robot['J3_max']))

        st.session_state['update_J1'].val = st.slider("J1", J1_min, J1_max, 0, 1, on_change=lambda: set_update('update_J1'))
        st.session_state['update_J2'].val = st.slider("J2", J2_min, J2_max, 0, 1, on_change=lambda: set_update('update_J2'))
        st.session_state['update_J3'].val = st.slider("J3", J3_min, J3_max, 0, 1, on_change=lambda: set_update('update_J3'))
        st.session_state['update_acc'].val = st.slider("acceleration", st.session_state.config_robot['a_min'], st.session_state.config_robot['a_max'], st.session_state.config_robot['base_acc_J1'], 1, on_change=lambda: set_update('update_acc'))

    st.subheader('Gripper')
    st.session_state['update_gripper'].val = st.slider("gripper", st.session_state.config_robot['gripper_min'], st.session_state.config_robot['gripper_max'], 0, 1, on_change=lambda: set_update('update_gripper'))

    # Home button doesn't need a value
    st.button("Home", on_click=lambda: set_update('update_home'))

    error_box = st.empty()

    for key in st.session_state.update_fns:
        if st.session_state[key].should_move:
            st.session_state.logger.LOG_DEBUG("Action: {key}, value: {st.session_state[key].val}") 
            res = st.session_state.update_fns[key](st.session_state[key].val)
            if not res:
                error_box.error("Invalid position given")

            # Since it's done moving should_move should be False
            st.session_state[key].should_move = False

def loop():
    # Enable normal mode of heatmap mode
    mode = st.sidebar.radio("Mode", ("Normal", "Heatmap", "G-code"))

    if mode == "Normal":
        normal_mode()
    elif mode == "Heatmap":
        heatmap_mode()
    elif mode == "G-code":
        gcode_mode()
    
