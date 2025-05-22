#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import mediapipe as mp
import numpy as np
import time
import yaml

# Assume camera and ohand_modbus_client are in the same directory or PYTHONPATH
from camera.orbbec_camera import OrbbecCamera, get_serial_numbers, close_connected_cameras

try:
    from ohand_modbus_client import OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX, FINGER_MIDDLE, FINGER_RING, \
        FINGER_PINKY, FINGER_THUMB_ROT
except ImportError:
    print("Error: ohand_modbus_client.py or its dependencies not found.")
    exit(1)

# --- Global Configuration Dictionaries ---
OHAND_INTERFACES = {
    'Left': {'config': {}, 'client': None, 'last_sent_time': 0, 'enabled': False, 'status_str': "Disabled"},
    'Right': {'config': {}, 'client': None, 'last_sent_time': 0, 'enabled': False, 'status_str': "Disabled"}
}
SHARED_VISUAL_INPUT_MAPPING = {}
MP_SETTINGS = {}
ORBBEC_SETTINGS = {}
GENERAL_SETTINGS = {}

# Landmark indices for angle calculation
JOINT_LIST = [
    [4, 3, 2],  # Thumb bend
    [8, 6, 5],  # Index bend
    [12, 10, 9],  # Middle bend
    [16, 14, 13],  # Ring bend
    [20, 18, 17]  # Pinky bend
]
FINGER_NAMES_SHORT = ["ThumbB", "IndexB", "MiddleB", "RingB", "PinkyB"]
THUMB_ROT_NAME_SHORT = "ThumbR"

# --- Text Drawing Configuration ---
FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE_SMALL = 0.45
FONT_SCALE_MEDIUM = 0.55
LINE_HEIGHT_SMALL = 18  # Increased for less overlap
LINE_HEIGHT_MEDIUM = 22  # Increased
TEXT_THICKNESS = 1
OUTLINE_COLOR = (0, 0, 0)  # Black outline
TITLE_TEXT_COLOR = (180, 220, 220)  # Light Cyan/Aqua for titles
DATA_TEXT_COLOR = (230, 240, 240)  # Very light off-white for data
STATUS_OK_COLOR = (100, 255, 100)  # Bright Green
STATUS_WARN_COLOR = (100, 180, 255)  # Light Orange/Yellow
STATUS_FAIL_COLOR = (100, 100, 255)  # Bright Red


def draw_text_with_outline(image, text, pos, font_face, scale, main_color, outline_color, thickness=1):
    """Draws text with a 1px black outline for better visibility."""
    x, y = pos
    # Draw outline (slightly offset)
    cv2.putText(image, text, (x + 1, y + 1), font_face, scale, outline_color, thickness, cv2.LINE_AA)
    # Draw main text
    cv2.putText(image, text, (x, y), font_face, scale, main_color, thickness, cv2.LINE_AA)


def load_config(config_path='dual_config.yaml'):
    """Loads configuration from YAML file."""
    global OHAND_INTERFACES, SHARED_VISUAL_INPUT_MAPPING, MP_SETTINGS, ORBBEC_SETTINGS, GENERAL_SETTINGS
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        for hand_label in ['Left', 'Right']:
            settings_key = f"ohand_{hand_label.lower()}_settings"
            mapping_key = f"{hand_label.lower()}_ohand_output_mapping"
            interface = OHAND_INTERFACES[hand_label]

            if settings_key in config:
                interface['config']['serial_port'] = config[settings_key].get('serial_port')
                interface['config']['slave_id'] = config[settings_key].get('slave_id')
                interface['enabled'] = config[settings_key].get('enabled', False)
                if interface['enabled']:
                    interface['status_str'] = "Enabled (No Port)" if not interface['config'][
                        'serial_port'] else "Enabled"
                else:
                    interface['status_str'] = "Disabled"
            else:  # Ensure status_str is set even if key is missing
                interface['status_str'] = "Config Missing"

            if mapping_key in config:
                interface['config'].update(config[mapping_key])
            else:
                print(
                    f"Warning: {mapping_key} not found in {config_path}. Using default empty ranges for {hand_label} hand.")
                interface['config'].setdefault('thumb_bend_range', (0, 0))
                interface['config'].setdefault('finger_bend_range', (0, 0))
                interface['config'].setdefault('thumb_rot_range', (0, 0))

        SHARED_VISUAL_INPUT_MAPPING = config.get('shared_visual_input_mapping', {})
        if not SHARED_VISUAL_INPUT_MAPPING:
            print(
                f"Warning: 'shared_visual_input_mapping' not found in {config_path}. Visual mapping may be inaccurate.")

        MP_SETTINGS = config.get('mediapipe_hands_settings', {})
        MP_SETTINGS.setdefault('min_detection_confidence', 0.7)
        MP_SETTINGS.setdefault('min_tracking_confidence', 0.6)
        MP_SETTINGS.setdefault('max_num_hands', 2)

        ORBBEC_SETTINGS = config.get('orbbec_camera_settings', {})
        if not ORBBEC_SETTINGS.get('serial_number'):
            print("Warning: Orbbec camera serial number not specified. Attempting auto-detection.")
            available_sns = get_serial_numbers()
            if available_sns:
                ORBBEC_SETTINGS['serial_number'] = available_sns[0]
                print(f"Auto-detected and using camera S/N: {ORBBEC_SETTINGS['serial_number']}")
            else:
                print("Error: No Orbbec cameras found. Specify S/N in config or connect a camera.")

        GENERAL_SETTINGS = config.get('general_settings', {})
        GENERAL_SETTINGS.setdefault('send_interval_seconds', 0.05)

        print("Dual hand control configuration loaded successfully.")
        print(
            f"Left Hand: {OHAND_INTERFACES['Left']['status_str']}, Right Hand: {OHAND_INTERFACES['Right']['status_str']}")

    except FileNotFoundError:
        print(f"Error: Configuration file {config_path} not found.")
        exit(1)
    # ... (rest of load_config exception handling) ...
    except yaml.YAMLError as e:
        print(f"Error parsing configuration file {config_path}: {e}")
        exit(1)
    except Exception as e:
        print(f"Unexpected error loading configuration: {e}")
        exit(1)


def translate_angle(value, visual_min, visual_max, ohand_min, ohand_max):
    """Linearly maps a value from a visual angle range to an OHand angle range."""
    if visual_max == visual_min:
        return ohand_min if value <= visual_min else ohand_max
    visual_span = visual_max - visual_min
    ohand_span = ohand_max - ohand_min
    value_scaled = float(value - visual_min) / float(visual_span)
    translated_value = ohand_min + (value_scaled * ohand_span)
    return max(min(translated_value, ohand_max), ohand_min)


def get_raw_thumb_rotation_deg(hand_landmarks):
    """Calculates the raw visual thumb rotation angle in degrees from hand landmarks."""
    WRIST, THUMB_CMC, THUMB_MCP, THUMB_TIP, INDEX_MCP = 0, 1, 2, 4, 5
    if not hand_landmarks: return 0
    lm = hand_landmarks.landmark
    vec_mcp_tip_2d = np.array([lm[THUMB_TIP].x, lm[THUMB_TIP].y]) - np.array([lm[THUMB_MCP].x, lm[THUMB_MCP].y])
    vec_wrist_index_mcp_2d = np.array([lm[INDEX_MCP].x, lm[INDEX_MCP].y]) - np.array([lm[WRIST].x, lm[WRIST].y])
    norm_mcp_tip = np.linalg.norm(vec_mcp_tip_2d)
    norm_wrist_index = np.linalg.norm(vec_wrist_index_mcp_2d)
    if norm_mcp_tip == 0 or norm_wrist_index == 0: return 0
    dot_product = np.dot(vec_mcp_tip_2d, vec_wrist_index_mcp_2d)
    value_for_acos = np.clip(dot_product / (norm_mcp_tip * norm_wrist_index), -1.0, 1.0)
    angle_rad = np.arccos(value_for_acos)
    angle_deg = np.degrees(angle_rad)
    cross_product = vec_mcp_tip_2d[0] * vec_wrist_index_mcp_2d[1] - vec_mcp_tip_2d[1] * vec_wrist_index_mcp_2d[0]
    if cross_product < 0: angle_deg = -angle_deg
    return int(angle_deg)


def get_hand_visual_angles(hand_landmarks):
    """Calculates raw and processed visual angles for a single hand."""
    processed_bend_angles = []
    raw_deg_bend_angles = []
    thumb_bend_proc_min = SHARED_VISUAL_INPUT_MAPPING.get('thumb_bend_input_min', 0)
    thumb_bend_proc_max = SHARED_VISUAL_INPUT_MAPPING.get('thumb_bend_input_max', 65)
    other_fingers_proc_min = SHARED_VISUAL_INPUT_MAPPING.get('other_fingers_bend_input_min', 0)
    other_fingers_proc_max = SHARED_VISUAL_INPUT_MAPPING.get('other_fingers_bend_input_max', 65)
    lm = hand_landmarks.landmark
    for i, joint_indices in enumerate(JOINT_LIST):
        a_coords = np.array([lm[joint_indices[0]].x, lm[joint_indices[0]].y])
        b_coords = np.array([lm[joint_indices[1]].x, lm[joint_indices[1]].y])
        c_coords = np.array([lm[joint_indices[2]].x, lm[joint_indices[2]].y])
        radians = np.arctan2(c_coords[1] - b_coords[1], c_coords[0] - b_coords[0]) - \
                  np.arctan2(a_coords[1] - b_coords[1], a_coords[0] - b_coords[0])
        raw_angle_deg = np.abs(radians * 180.0 / np.pi)
        if raw_angle_deg > 180.0: raw_angle_deg = 360.0 - raw_angle_deg
        raw_deg_bend_angles.append(int(raw_angle_deg))
        processed_angle = 0
        if i == 0:
            clamped_raw_for_interp = max(90, min(raw_angle_deg, 180))
            processed_angle = np.interp(clamped_raw_for_interp, [90, 180], [0, 200])
            processed_angle = max(thumb_bend_proc_min, min(processed_angle, thumb_bend_proc_max))
        else:
            clamped_raw_for_interp = max(30, min(raw_angle_deg, 180))
            processed_angle = np.interp(clamped_raw_for_interp, [30, 180], [0, 180])
            processed_angle = max(other_fingers_proc_min, min(processed_angle, other_fingers_proc_max))
        processed_bend_angles.append(int(processed_angle))
    processed_thumb_rot_angle_deg = get_raw_thumb_rotation_deg(hand_landmarks)
    return {
        'proc_bends': processed_bend_angles,
        'raw_deg_bends': raw_deg_bend_angles,
        'proc_rot_deg': processed_thumb_rot_angle_deg
    }


def initialize_ohand_clients():
    """Initializes OHand clients based on loaded configuration."""
    for hand_label, interface in OHAND_INTERFACES.items():
        if interface['enabled'] and interface['config'].get('serial_port'):
            print(
                f"Initializing {hand_label} OHand: Port {interface['config']['serial_port']}, ID {interface['config']['slave_id']}")
            client = OHandModbusClient(port=interface['config']['serial_port'],
                                       slave_id=interface['config']['slave_id'])
            if client.connect():
                interface['client'] = client
                interface['status_str'] = f"Connected ({interface['config']['serial_port']})"
                print(f"{hand_label} OHand connected successfully.")
            else:
                interface['status_str'] = f"Connect Fail ({interface['config']['serial_port']})"
                print(f"Failed to connect to {hand_label} OHand. It will not be controlled.")
                interface['enabled'] = False
        # Status string for other cases already set in load_config


def draw_info_panel(image, fps, hand_info_cache):
    """Draws the information panel on the image."""
    panel_bg_color = (40, 40, 40)  # Slightly darker panel background
    img_h, img_w = image.shape[:2]

    # --- Top Global Info Panel ---
    top_panel_h = 75
    cv2.rectangle(image, (0, 0), (img_w, top_panel_h), panel_bg_color, -1)
    cv2.rectangle(image, (0, top_panel_h), (img_w, top_panel_h + 1), (80, 80, 80), 1)  # Line separator

    draw_text_with_outline(image, f"FPS: {fps:.1f}", (10, 25), FONT_FACE, FONT_SCALE_MEDIUM, DATA_TEXT_COLOR,
                           OUTLINE_COLOR, TEXT_THICKNESS)
    cam_sn = ORBBEC_SETTINGS.get('serial_number', "N/A")
    draw_text_with_outline(image, f"Cam S/N: {cam_sn}", (10, 25 + LINE_HEIGHT_MEDIUM), FONT_FACE, FONT_SCALE_MEDIUM,
                           DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)

    l_status_str = OHAND_INTERFACES['Left']['status_str']
    l_status_color = STATUS_OK_COLOR if "Connected" in l_status_str else (
        STATUS_WARN_COLOR if "Enabled" in l_status_str else STATUS_FAIL_COLOR)
    draw_text_with_outline(image, f"L-OHand: {l_status_str}", (250, 25), FONT_FACE, FONT_SCALE_MEDIUM, l_status_color,
                           OUTLINE_COLOR, TEXT_THICKNESS)

    r_status_str = OHAND_INTERFACES['Right']['status_str']
    r_status_color = STATUS_OK_COLOR if "Connected" in r_status_str else (
        STATUS_WARN_COLOR if "Enabled" in r_status_str else STATUS_FAIL_COLOR)
    draw_text_with_outline(image, f"R-OHand: {r_status_str}", (250, 25 + LINE_HEIGHT_MEDIUM), FONT_FACE,
                           FONT_SCALE_MEDIUM, r_status_color, OUTLINE_COLOR, TEXT_THICKNESS)

    # --- Per-Hand Info Panels (Left Side) ---
    start_y_hands_panel = top_panel_h + 20
    col_width = 220  # Adjusted column width
    section_gap = 8  # Small gap between sections (Raw, Processed, Cmd)

    for i, hand_label in enumerate(['Left', 'Right']):
        if hand_label not in hand_info_cache or not hand_info_cache[hand_label]:
            # Optionally draw a placeholder if hand not detected but configured
            if OHAND_INTERFACES[hand_label]['enabled']:
                start_x_placeholder = 10 + i * (col_width + 10)
                current_y_placeholder = start_y_hands_panel
                draw_text_with_outline(image, f"{hand_label} Hand (Not Detected)",
                                       (start_x_placeholder, current_y_placeholder), FONT_FACE, FONT_SCALE_MEDIUM,
                                       STATUS_WARN_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
            continue

        info = hand_info_cache[hand_label]
        start_x = 10 + i * (col_width + 10)
        current_y = start_y_hands_panel

        # Hand Label and Status
        hand_title_color = STATUS_OK_COLOR if info.get('status') == "Controlled" else STATUS_WARN_COLOR
        draw_text_with_outline(image, f"{info.get('label', hand_label)} ({info.get('status', 'N/A')})",
                               (start_x, current_y), FONT_FACE, FONT_SCALE_MEDIUM, hand_title_color, OUTLINE_COLOR,
                               TEXT_THICKNESS)
        current_y += LINE_HEIGHT_MEDIUM
        draw_text_with_outline(image, f"  Conf: {info.get('score', 0.0):.2f}",
                               (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR,
                               TEXT_THICKNESS)
        current_y += LINE_HEIGHT_SMALL + section_gap

        if info.get('status') == "Controlled":
            # Raw Visual Angles
            draw_text_with_outline(image, "Raw Visual (deg):", (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL,
                                   TITLE_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
            current_y += LINE_HEIGHT_SMALL
            raw_bends = info.get('raw_deg_bends', [])
            raw_rot = info.get('proc_rot_deg', 0)
            for j, name in enumerate(FINGER_NAMES_SHORT):
                val = raw_bends[j] if j < len(raw_bends) else "N/A"
                draw_text_with_outline(image, f"  {name}: {val}", (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL,
                                       DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
                current_y += LINE_HEIGHT_SMALL
            draw_text_with_outline(image, f"  {THUMB_ROT_NAME_SHORT}: {raw_rot}", (start_x, current_y), FONT_FACE,
                                   FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
            current_y += LINE_HEIGHT_SMALL + section_gap

            # Processed Visual Angles
            draw_text_with_outline(image, "Processed Visual (Input):", (start_x, current_y), FONT_FACE,
                                   FONT_SCALE_SMALL, TITLE_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
            current_y += LINE_HEIGHT_SMALL
            proc_bends = info.get('proc_bends', [])
            proc_rot = info.get('proc_rot_deg', 0)
            for j, name in enumerate(FINGER_NAMES_SHORT):
                val = proc_bends[j] if j < len(proc_bends) else "N/A"
                draw_text_with_outline(image, f"  {name}: {val}", (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL,
                                       DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
                current_y += LINE_HEIGHT_SMALL
            draw_text_with_outline(image, f"  {THUMB_ROT_NAME_SHORT}: {proc_rot}", (start_x, current_y), FONT_FACE,
                                   FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
            current_y += LINE_HEIGHT_SMALL + section_gap

            # Target OHand Angles
            draw_text_with_outline(image, "Target OHand Cmd (deg):", (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL,
                                   TITLE_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
            current_y += LINE_HEIGHT_SMALL
            ohand_cmds = info.get('ohand_cmds', {})
            cmd_angles_map = {
                FINGER_THUMB_BEND: FINGER_NAMES_SHORT[0], FINGER_INDEX: FINGER_NAMES_SHORT[1],
                FINGER_MIDDLE: FINGER_NAMES_SHORT[2], FINGER_RING: FINGER_NAMES_SHORT[3],
                FINGER_PINKY: FINGER_NAMES_SHORT[4], FINGER_THUMB_ROT: THUMB_ROT_NAME_SHORT
            }
            for finger_const, name_short in cmd_angles_map.items():
                val_str = f"{ohand_cmds.get(finger_const, 0.0):.1f}" if ohand_cmds else "N/A"
                draw_text_with_outline(image, f"  {name_short}: {val_str}", (start_x, current_y), FONT_FACE,
                                       FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
                current_y += LINE_HEIGHT_SMALL


# --- Main Function ---
def main():
    load_config()
    initialize_ohand_clients()

    mp_hands_sol = mp.solutions.hands
    hands_detector = mp_hands_sol.Hands(
        max_num_hands=MP_SETTINGS.get('max_num_hands', 2),
        min_detection_confidence=MP_SETTINGS.get('min_detection_confidence', 0.7),
        min_tracking_confidence=MP_SETTINGS.get('min_tracking_confidence', 0.6)
    )
    mp_drawing_sol = mp.solutions.drawing_utils
    mp_drawing_styles_sol = mp.solutions.drawing_styles

    print(f"Initializing Orbbec camera, S/N: {ORBBEC_SETTINGS.get('serial_number', 'Auto-detect')}...")
    camera = None
    try:
        camera = OrbbecCamera(ORBBEC_SETTINGS.get('serial_number'))
        camera.start_stream(depth_stream=True, color_stream=True, enable_sync=False, use_alignment=False)
        print("Orbbec camera stream started successfully.")
    except Exception as e:
        print(f"Error: Failed to initialize or start Orbbec camera: {e}")
        for hand_label in ['Left', 'Right']:  # Cleanup attempt
            if OHAND_INTERFACES[hand_label]['client'] and OHAND_INTERFACES[hand_label]['client'].is_connected:
                OHAND_INTERFACES[hand_label]['client'].disconnect()
        return

    print("Dual hand tracking started. Press 'q' to quit.")
    send_interval_sec = GENERAL_SETTINGS.get('send_interval_seconds', 0.05)
    prev_fps_time = time.time()
    fps = 0.0
    hand_info_cache_current_frame = {}

    try:
        while True:
            frame_start_time = time.time()
            color_img_bgr, _, _ = camera.get_frames()
            if color_img_bgr is None:
                print("Error: Failed to get frame from Orbbec camera.")
                time.sleep(0.1);
                continue

            current_fps_time = time.time()
            if (current_fps_time - prev_fps_time) > 1e-9:  # Avoid division by zero
                fps = 1.0 / (current_fps_time - prev_fps_time)
            prev_fps_time = current_fps_time

            img_rgb_flipped = cv2.cvtColor(cv2.flip(color_img_bgr, 1), cv2.COLOR_BGR2RGB)
            img_rgb_flipped.flags.writeable = False
            results_mp = hands_detector.process(img_rgb_flipped)
            img_rgb_flipped.flags.writeable = True
            output_display_img = cv2.cvtColor(img_rgb_flipped, cv2.COLOR_RGB2BGR)

            hand_info_cache_current_frame.clear()
            commands_to_send_this_frame = []

            if results_mp.multi_hand_landmarks:
                for idx, hand_landmarks_mp in enumerate(results_mp.multi_hand_landmarks):
                    handedness_mp = results_mp.multi_handedness[idx].classification[0]
                    hand_label_detected = handedness_mp.label

                    current_hand_frame_info = {
                        'label': hand_label_detected, 'score': handedness_mp.score,
                        'status': "Not Controlled", 'raw_deg_bends': [], 'proc_bends': [],
                        'proc_rot_deg': 0, 'ohand_cmds': {}
                    }
                    interface = OHAND_INTERFACES[hand_label_detected]
                    wrist_lm = hand_landmarks_mp.landmark[0]  # For placing hand label
                    hand_label_pos = (int(wrist_lm.x * output_display_img.shape[1]),
                                      int(wrist_lm.y * output_display_img.shape[0] - 20))  # slightly above wrist

                    if not interface['enabled'] or not interface['client']:
                        mp_drawing_sol.draw_landmarks(
                            output_display_img, hand_landmarks_mp, mp_hands_sol.HAND_CONNECTIONS,
                            mp_drawing_styles_sol.get_default_hand_landmarks_style(),
                            mp_drawing_styles_sol.get_default_hand_connections_style())
                        draw_text_with_outline(output_display_img, f"{hand_label_detected} (Not Controlled)",
                                               hand_label_pos, FONT_FACE, FONT_SCALE_MEDIUM, STATUS_WARN_COLOR,
                                               OUTLINE_COLOR, TEXT_THICKNESS)
                    else:
                        current_hand_frame_info['status'] = "Controlled"
                        landmark_draw_color = STATUS_OK_COLOR if hand_label_detected == "Right" else (
                        255, 150, 150)  # R:Green, L:LightBlue
                        mp_drawing_sol.draw_landmarks(
                            output_display_img, hand_landmarks_mp, mp_hands_sol.HAND_CONNECTIONS,
                            mp_drawing_sol.DrawingSpec(color=landmark_draw_color, thickness=2, circle_radius=3),
                            mp_drawing_sol.DrawingSpec(color=landmark_draw_color, thickness=1, circle_radius=1))
                        draw_text_with_outline(output_display_img, f"{hand_label_detected} Hand",
                                               hand_label_pos, FONT_FACE, FONT_SCALE_MEDIUM, landmark_draw_color,
                                               OUTLINE_COLOR, TEXT_THICKNESS)

                        visual_angles = get_hand_visual_angles(hand_landmarks_mp)
                        current_hand_frame_info.update(visual_angles)
                        ohand_target_cmds = {}
                        hand_ohand_cfg = interface['config']
                        proc_bends_list = visual_angles['proc_bends']

                        if len(proc_bends_list) == 5:
                            vis_thumb_b_min = SHARED_VISUAL_INPUT_MAPPING.get('thumb_bend_input_min', 0);
                            vis_thumb_b_max = SHARED_VISUAL_INPUT_MAPPING.get('thumb_bend_input_max', 65)
                            ohand_target_cmds[FINGER_THUMB_BEND] = translate_angle(proc_bends_list[0], vis_thumb_b_min,
                                                                                   vis_thumb_b_max,
                                                                                   hand_ohand_cfg['thumb_bend_range'][
                                                                                       0],
                                                                                   hand_ohand_cfg['thumb_bend_range'][
                                                                                       1])
                            vis_other_b_min = SHARED_VISUAL_INPUT_MAPPING.get('other_fingers_bend_input_min', 0);
                            vis_other_b_max = SHARED_VISUAL_INPUT_MAPPING.get('other_fingers_bend_input_max', 65)
                            ohand_f_b_range = hand_ohand_cfg['finger_bend_range']
                            ohand_target_cmds[FINGER_INDEX] = translate_angle(proc_bends_list[1], vis_other_b_min,
                                                                              vis_other_b_max, ohand_f_b_range[0],
                                                                              ohand_f_b_range[1])
                            ohand_target_cmds[FINGER_MIDDLE] = translate_angle(proc_bends_list[2], vis_other_b_min,
                                                                               vis_other_b_max, ohand_f_b_range[0],
                                                                               ohand_f_b_range[1])
                            ohand_target_cmds[FINGER_RING] = translate_angle(proc_bends_list[3], vis_other_b_min,
                                                                             vis_other_b_max, ohand_f_b_range[0],
                                                                             ohand_f_b_range[1])
                            ohand_target_cmds[FINGER_PINKY] = translate_angle(proc_bends_list[4], vis_other_b_min,
                                                                              vis_other_b_max, ohand_f_b_range[0],
                                                                              ohand_f_b_range[1])

                        proc_rot_val = visual_angles['proc_rot_deg']
                        vis_thumb_r_min = SHARED_VISUAL_INPUT_MAPPING.get('thumb_rot_input_min', -90);
                        vis_thumb_r_max = SHARED_VISUAL_INPUT_MAPPING.get('thumb_rot_input_max', 90)
                        ohand_target_cmds[FINGER_THUMB_ROT] = translate_angle(proc_rot_val, vis_thumb_r_min,
                                                                              vis_thumb_r_max,
                                                                              hand_ohand_cfg['thumb_rot_range'][0],
                                                                              hand_ohand_cfg['thumb_rot_range'][1])
                        current_hand_frame_info['ohand_cmds'] = ohand_target_cmds

                        if ohand_target_cmds and (frame_start_time - interface['last_sent_time'] > send_interval_sec):
                            commands_to_send_this_frame.append(
                                {'label': hand_label_detected, 'commands': ohand_target_cmds})
                            interface['last_sent_time'] = frame_start_time
                    hand_info_cache_current_frame[hand_label_detected] = current_hand_frame_info

            draw_info_panel(output_display_img, fps, hand_info_cache_current_frame)

            if commands_to_send_this_frame:
                # console_output = f"--- Frame @ {frame_start_time:.2f} (FPS: {fps:.1f}) ---\n" # Reduce console clutter
                for item_to_send in commands_to_send_this_frame:
                    hand_lbl = item_to_send['label'];
                    cmds = item_to_send['commands']
                    client_instance = OHAND_INTERFACES[hand_lbl]['client']
                    # console_output += f"  Sending to {hand_lbl} OHand:\n"
                    for finger_idx_const, angle_val_cmd in cmds.items():
                        # console_output += f"    FingerID {finger_idx_const}: {angle_val_cmd:.1f} deg\n"
                        try:
                            client_instance.set_finger_target_angle(finger_idx_const, angle_val_cmd)
                        except Exception as e:
                            print(f"    Error setting {hand_lbl} finger {finger_idx_const}: {e}")
                # console_output += "-" * 25
                # print(console_output) # Print once per frame batch

            cv2.imshow('Dual OHand Visual Control with Info', output_display_img)
            if cv2.waitKey(5) & 0xFF == ord('q'):
                print("Quit command received. Exiting...");
                break

    except KeyboardInterrupt:
        print("User interrupted (Ctrl+C). Exiting...")
    except Exception as e:
        print(f"An unexpected error occurred in the main loop: {e}");
        import traceback;
        traceback.print_exc()
    finally:
        print("Cleaning up resources...")
        if camera:
            try:
                camera.stop_stream(); print("Camera stream stopped.")
            except Exception as e:
                print(f"Error stopping camera stream: {e}")
        cv2.destroyAllWindows();
        print("OpenCV windows closed.")
        for hand_label_cleanup in ['Left', 'Right']:
            interface_cleanup = OHAND_INTERFACES[hand_label_cleanup]
            if interface_cleanup['client'] and interface_cleanup['client'].is_connected:
                print(f"Safely positioning and disconnecting {hand_label_cleanup} OHand...")
                # ... (rest of cleanup as before) ...
                try:
                    cfg_cleanup = interface_cleanup['config']
                    safe_open_angle = (cfg_cleanup.get('finger_bend_range', [0, 100])[0] +
                                       cfg_cleanup.get('finger_bend_range', [0, 100])[1]) / 15
                    mid_rot_angle = (cfg_cleanup.get('thumb_rot_range', [-45, 45])[0] +
                                     cfg_cleanup.get('thumb_rot_range', [-45, 45])[1]) / 2
                    client_to_cleanup = interface_cleanup['client']
                    client_to_cleanup.set_finger_target_angle(FINGER_THUMB_BEND, safe_open_angle)
                    client_to_cleanup.set_finger_target_angle(FINGER_INDEX, safe_open_angle);
                    client_to_cleanup.set_finger_target_angle(FINGER_MIDDLE, safe_open_angle)
                    client_to_cleanup.set_finger_target_angle(FINGER_RING, safe_open_angle);
                    client_to_cleanup.set_finger_target_angle(FINGER_PINKY, safe_open_angle)
                    client_to_cleanup.set_finger_target_angle(FINGER_THUMB_ROT, mid_rot_angle)
                    time.sleep(0.3)
                except Exception as e_pos:
                    print(f"  Error during safe positioning for {hand_label_cleanup} OHand: {e_pos}")
                finally:
                    try:
                        interface_cleanup['client'].disconnect(); print(f"  {hand_label_cleanup} OHand disconnected.")
                    except Exception as e_dc:
                        print(f"  Error disconnecting {hand_label_cleanup} OHand: {e_dc}")
        print("Cleanup complete.")


if __name__ == '__main__':
    main()