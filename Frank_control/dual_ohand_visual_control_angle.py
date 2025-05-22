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
    'Left': {'config': {'visual_input_mapping': {}}, 'client': None, 'last_sent_time': 0, 'enabled': False,
             'status_str': "Disabled"},
    'Right': {'config': {'visual_input_mapping': {}}, 'client': None, 'last_sent_time': 0, 'enabled': False,
              'status_str': "Disabled"}
}
# SHARED_VISUAL_INPUT_MAPPING is no longer used directly for processing,
# it will be loaded into each hand's config.
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
LINE_HEIGHT_SMALL = 18
LINE_HEIGHT_MEDIUM = 22
TEXT_THICKNESS = 1
OUTLINE_COLOR = (0, 0, 0)
TITLE_TEXT_COLOR = (180, 220, 220)
DATA_TEXT_COLOR = (230, 240, 240)
STATUS_OK_COLOR = (100, 255, 100)
STATUS_WARN_COLOR = (100, 180, 255)
STATUS_FAIL_COLOR = (100, 100, 255)


def draw_text_with_outline(image, text, pos, font_face, scale, main_color, outline_color, thickness=1):
    x, y = pos
    cv2.putText(image, text, (x + 1, y + 1), font_face, scale, outline_color, thickness, cv2.LINE_AA)
    cv2.putText(image, text, (x, y), font_face, scale, main_color, thickness, cv2.LINE_AA)


def load_config(config_path='dual_config.yaml'):
    """Loads configuration from YAML file."""
    global OHAND_INTERFACES, MP_SETTINGS, ORBBEC_SETTINGS, GENERAL_SETTINGS  # SHARED_VISUAL_INPUT_MAPPING removed from globals
    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        for hand_label in ['Left', 'Right']:
            settings_key = f"ohand_{hand_label.lower()}_settings"
            ohand_output_mapping_key = f"{hand_label.lower()}_ohand_output_mapping"
            visual_input_mapping_key = f"{hand_label.lower()}_visual_input_mapping"  # New key for visual input
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
            else:
                interface['status_str'] = "Config Missing"

            # Load OHand output mapping
            if ohand_output_mapping_key in config:
                interface['config'].update(config[ohand_output_mapping_key])  # Adds thumb_bend_range etc.
            else:
                print(
                    f"Warning: {ohand_output_mapping_key} not found in {config_path}. Using default empty OHand output ranges for {hand_label} hand.")
                interface['config'].setdefault('thumb_bend_range', (0, 0))
                interface['config'].setdefault('finger_bend_range', (0, 0))
                interface['config'].setdefault('thumb_rot_range', (0, 0))

            # Load hand-specific visual input mapping
            if visual_input_mapping_key in config:
                interface['config']['visual_input_mapping'] = config[visual_input_mapping_key]
            else:
                print(
                    f"Warning: {visual_input_mapping_key} not found for {hand_label} hand in {config_path}. Using empty visual input map.")
                interface['config']['visual_input_mapping'] = {}

        MP_SETTINGS = config.get('mediapipe_hands_settings', {})
        MP_SETTINGS.setdefault('min_detection_confidence', 0.7)
        MP_SETTINGS.setdefault('min_tracking_confidence', 0.6)
        MP_SETTINGS.setdefault('max_num_hands', 2)

        ORBBEC_SETTINGS = config.get('orbbec_camera_settings', {})
        if not ORBBEC_SETTINGS.get('serial_number'):
            print("Warning: Orbbec camera serial number not specified. Attempting auto-detection.")
            available_sns = get_serial_numbers()
            if available_sns:
                ORBBEC_SETTINGS['serial_number'] = available_sns[0]; print(
                    f"Auto-detected Cam S/N: {ORBBEC_SETTINGS['serial_number']}")
            else:
                print("Error: No Orbbec cameras found.")

        GENERAL_SETTINGS = config.get('general_settings', {})
        GENERAL_SETTINGS.setdefault('send_interval_seconds', 0.05)

        print("Dual hand control configuration loaded.")
        for hand_label in ['Left', 'Right']:
            print(f"  {hand_label} Hand: Status='{OHAND_INTERFACES[hand_label]['status_str']}', "
                  f"VisualInputMap Loaded: {'Yes' if OHAND_INTERFACES[hand_label]['config']['visual_input_mapping'] else 'No'}")


    except FileNotFoundError:
        print(f"Error: Config file {config_path} not found."); exit(1)
    except yaml.YAMLError as e:
        print(f"Error parsing config file {config_path}: {e}"); exit(1)
    except Exception as e:
        print(f"Unexpected error loading config: {e}"); exit(1)


def translate_angle(value, visual_min, visual_max, ohand_min, ohand_max):
    if visual_max == visual_min: return ohand_min if value <= visual_min else ohand_max  # Handles min == max or min > max case by clamping
    # If visual_min > visual_max, the mapping is inverted.
    visual_span = visual_max - visual_min  # This will be negative if min > max
    ohand_span = ohand_max - ohand_min
    # Clamping value to the visual range before scaling
    clamped_value = max(min(value, visual_max), visual_min) if visual_min <= visual_max else max(min(value, visual_min),
                                                                                                 visual_max)

    value_scaled = float(clamped_value - visual_min) / float(visual_span)
    translated_value = ohand_min + (value_scaled * ohand_span)
    return max(min(translated_value, ohand_max), ohand_min)


def get_raw_thumb_rotation_deg(hand_landmarks):
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


def get_hand_visual_angles(hand_landmarks, hand_visual_input_map):  # Added hand_visual_input_map argument
    """Calculates raw and processed visual angles for a single hand, using hand-specific visual input map for clamping."""
    processed_bend_angles = []
    raw_deg_bend_angles = []

    # Get hand-specific visual input parameters for clamping processed angles
    # Provide defaults if keys are missing in the specific map
    thumb_bend_proc_min = hand_visual_input_map.get('thumb_bend_input_min', 0)
    thumb_bend_proc_max = hand_visual_input_map.get('thumb_bend_input_max', 65)
    other_fingers_proc_min = hand_visual_input_map.get('other_fingers_bend_input_min', 0)
    other_fingers_proc_max = hand_visual_input_map.get('other_fingers_bend_input_max', 65)

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
        if i == 0:  # Thumb bend
            clamped_raw_for_interp = max(90, min(raw_angle_deg, 180))
            processed_angle = np.interp(clamped_raw_for_interp, [90, 180], [0, 200])  # Initial interpolation
            # Clamp to final processed visual range from hand-specific config
            processed_angle = max(min(processed_angle, thumb_bend_proc_max),
                                  thumb_bend_proc_min) if thumb_bend_proc_min <= thumb_bend_proc_max \
                else max(min(processed_angle, thumb_bend_proc_min), thumb_bend_proc_max)  # Handle inverted range
        else:  # Other finger bends
            clamped_raw_for_interp = max(30, min(raw_angle_deg, 180))
            processed_angle = np.interp(clamped_raw_for_interp, [30, 180], [0, 180])
            processed_angle = max(min(processed_angle, other_fingers_proc_max),
                                  other_fingers_proc_min) if other_fingers_proc_min <= other_fingers_proc_max \
                else max(min(processed_angle, other_fingers_proc_min), other_fingers_proc_max)
        processed_bend_angles.append(int(processed_angle))
    processed_thumb_rot_angle_deg = get_raw_thumb_rotation_deg(hand_landmarks)
    return {
        'proc_bends': processed_bend_angles,
        'raw_deg_bends': raw_deg_bend_angles,
        'proc_rot_deg': processed_thumb_rot_angle_deg
    }


def initialize_ohand_clients():
    for hand_label, interface in OHAND_INTERFACES.items():
        if interface['enabled'] and interface['config'].get('serial_port'):
            print(
                f"Initializing {hand_label} OHand: Port {interface['config']['serial_port']}, ID {interface['config']['slave_id']}")
            client = OHandModbusClient(port=interface['config']['serial_port'],
                                       slave_id=interface['config']['slave_id'])
            if client.connect():
                interface['client'] = client;
                interface['status_str'] = f"Connected ({interface['config']['serial_port']})"
                print(f"{hand_label} OHand connected.")
            else:
                interface['status_str'] = f"Connect Fail ({interface['config']['serial_port']})";
                print(f"Failed to connect {hand_label} OHand.");
                interface['enabled'] = False
        # Other status strings already set in load_config


def draw_info_panel(image, fps, hand_info_cache):
    panel_bg_color = (40, 40, 40);
    img_h, img_w = image.shape[:2]
    top_panel_h = 75
    cv2.rectangle(image, (0, 0), (img_w, top_panel_h), panel_bg_color, -1)
    cv2.rectangle(image, (0, top_panel_h), (img_w, top_panel_h + 1), (80, 80, 80), 1)
    draw_text_with_outline(image, f"FPS: {fps:.1f}", (10, 25), FONT_FACE, FONT_SCALE_MEDIUM, DATA_TEXT_COLOR,
                           OUTLINE_COLOR, TEXT_THICKNESS)
    cam_sn = ORBBEC_SETTINGS.get('serial_number', "N/A")
    draw_text_with_outline(image, f"Cam S/N: {cam_sn}", (10, 25 + LINE_HEIGHT_MEDIUM), FONT_FACE, FONT_SCALE_MEDIUM,
                           DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
    l_status_str = OHAND_INTERFACES['Left']['status_str'];
    l_s_color = STATUS_OK_COLOR if "Connected" in l_status_str else (
        STATUS_WARN_COLOR if "Enabled" in l_status_str else STATUS_FAIL_COLOR)
    draw_text_with_outline(image, f"L-OHand: {l_status_str}", (280, 25), FONT_FACE, FONT_SCALE_MEDIUM, l_s_color,
                           OUTLINE_COLOR, TEXT_THICKNESS)  # Adjusted X for L/R status
    r_status_str = OHAND_INTERFACES['Right']['status_str'];
    r_s_color = STATUS_OK_COLOR if "Connected" in r_status_str else (
        STATUS_WARN_COLOR if "Enabled" in r_status_str else STATUS_FAIL_COLOR)
    draw_text_with_outline(image, f"R-OHand: {r_status_str}", (280, 25 + LINE_HEIGHT_MEDIUM), FONT_FACE,
                           FONT_SCALE_MEDIUM, r_s_color, OUTLINE_COLOR, TEXT_THICKNESS)

    start_y_hands_panel = top_panel_h + 20;
    col_width = 220;
    section_gap = 8
    for i, hand_label in enumerate(['Left', 'Right']):
        start_x = 10 + i * (col_width + 10);
        current_y = start_y_hands_panel
        if hand_label not in hand_info_cache or not hand_info_cache[hand_label]:
            if OHAND_INTERFACES[hand_label]['enabled']:
                draw_text_with_outline(image, f"{hand_label} Hand (Not Detected)", (start_x, current_y), FONT_FACE,
                                       FONT_SCALE_MEDIUM, STATUS_WARN_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
            continue
        info = hand_info_cache[hand_label]
        hand_title_color = STATUS_OK_COLOR if info.get('status') == "Controlled" else STATUS_WARN_COLOR
        draw_text_with_outline(image, f"{info.get('label', hand_label)} ({info.get('status', 'N/A')})",
                               (start_x, current_y), FONT_FACE, FONT_SCALE_MEDIUM, hand_title_color, OUTLINE_COLOR,
                               TEXT_THICKNESS)
        current_y += LINE_HEIGHT_MEDIUM
        draw_text_with_outline(image, f"  Conf: {info.get('score', 0.0):.2f}", (start_x, current_y), FONT_FACE,
                               FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS)
        current_y += LINE_HEIGHT_SMALL + section_gap
        if info.get('status') == "Controlled":
            vis_map_for_hand = OHAND_INTERFACES[hand_label]['config'].get('visual_input_mapping',
                                                                          {})  # For displaying input ranges

            draw_text_with_outline(image, f"Raw Visual (deg):", (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL,
                                   TITLE_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS);
            current_y += LINE_HEIGHT_SMALL
            raw_bends = info.get('raw_deg_bends', []);
            raw_rot = info.get('proc_rot_deg', 0)
            for j, name in enumerate(FINGER_NAMES_SHORT): draw_text_with_outline(image,
                                                                                 f"  {name}: {raw_bends[j] if j < len(raw_bends) else 'N/A'}",
                                                                                 (start_x, current_y), FONT_FACE,
                                                                                 FONT_SCALE_SMALL, DATA_TEXT_COLOR,
                                                                                 OUTLINE_COLOR,
                                                                                 TEXT_THICKNESS); current_y += LINE_HEIGHT_SMALL
            draw_text_with_outline(image, f"  {THUMB_ROT_NAME_SHORT}: {raw_rot}", (start_x, current_y), FONT_FACE,
                                   FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS);
            current_y += LINE_HEIGHT_SMALL + section_gap

            draw_text_with_outline(image, f"Processed Visual (Input):", (start_x, current_y), FONT_FACE,
                                   FONT_SCALE_SMALL, TITLE_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS);
            current_y += LINE_HEIGHT_SMALL
            proc_bends = info.get('proc_bends', []);
            proc_rot = info.get('proc_rot_deg', 0)
            for j, name in enumerate(FINGER_NAMES_SHORT):
                v_min = vis_map_for_hand.get('thumb_bend_input_min' if j == 0 else 'other_fingers_bend_input_min', 'N')
                v_max = vis_map_for_hand.get('thumb_bend_input_max' if j == 0 else 'other_fingers_bend_input_max', 'A')
                text = f"  {name}: {proc_bends[j] if j < len(proc_bends) else 'N/A'} (Rng:[{v_min},{v_max}])"
                draw_text_with_outline(image, text, (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL, DATA_TEXT_COLOR,
                                       OUTLINE_COLOR, TEXT_THICKNESS);
                current_y += LINE_HEIGHT_SMALL
            v_min_rot = vis_map_for_hand.get('thumb_rot_input_min', 'N');
            v_max_rot = vis_map_for_hand.get('thumb_rot_input_max', 'A')
            draw_text_with_outline(image, f"  {THUMB_ROT_NAME_SHORT}: {proc_rot} (Rng:[{v_min_rot},{v_max_rot}])",
                                   (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR,
                                   TEXT_THICKNESS);
            current_y += LINE_HEIGHT_SMALL + section_gap

            draw_text_with_outline(image, "Target OHand Cmd (deg):", (start_x, current_y), FONT_FACE, FONT_SCALE_SMALL,
                                   TITLE_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS);
            current_y += LINE_HEIGHT_SMALL
            ohand_cmds = info.get('ohand_cmds', {});
            cmd_angles_map = {FINGER_THUMB_BEND: FINGER_NAMES_SHORT[0], FINGER_INDEX: FINGER_NAMES_SHORT[1],
                              FINGER_MIDDLE: FINGER_NAMES_SHORT[2], FINGER_RING: FINGER_NAMES_SHORT[3],
                              FINGER_PINKY: FINGER_NAMES_SHORT[4], FINGER_THUMB_ROT: THUMB_ROT_NAME_SHORT}
            for finger_const, name_short in cmd_angles_map.items():
                val_str = f"{ohand_cmds.get(finger_const, 0.0):.1f}" if ohand_cmds else "N/A"
                draw_text_with_outline(image, f"  {name_short}: {val_str}", (start_x, current_y), FONT_FACE,
                                       FONT_SCALE_SMALL, DATA_TEXT_COLOR, OUTLINE_COLOR, TEXT_THICKNESS);
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
        print("Orbbec camera stream started.")
    except Exception as e:
        print(f"Error: Failed to init/start Orbbec camera: {e}")
        for hand_label in ['Left', 'Right']:
            if OHAND_INTERFACES[hand_label]['client'] and OHAND_INTERFACES[hand_label]['client'].is_connected:
                OHAND_INTERFACES[hand_label]['client'].disconnect()
        return

    print("Dual hand tracking started. Press 'q' to quit.")
    send_interval_sec = GENERAL_SETTINGS.get('send_interval_seconds', 0.05)
    prev_fps_time = time.time();
    fps = 0.0
    hand_info_cache_current_frame = {}

    try:
        while True:
            frame_start_time = time.time()
            color_img_bgr, _, _ = camera.get_frames()
            if color_img_bgr is None: print("Error: Failed to get frame."); time.sleep(0.1); continue

            current_fps_time = time.time()
            if (current_fps_time - prev_fps_time) > 1e-9: fps = 1.0 / (current_fps_time - prev_fps_time)
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

                    current_hand_frame_info = {'label': hand_label_detected, 'score': handedness_mp.score,
                                               'status': "Not Controlled", 'raw_deg_bends': [], 'proc_bends': [],
                                               'proc_rot_deg': 0, 'ohand_cmds': {}}
                    interface = OHAND_INTERFACES[hand_label_detected]
                    # Get hand-specific visual input mapping (or empty dict if not configured)
                    current_hand_visual_map = interface['config'].get('visual_input_mapping', {})

                    wrist_lm = hand_landmarks_mp.landmark[0]
                    hand_label_pos = (
                    int(wrist_lm.x * output_display_img.shape[1]), int(wrist_lm.y * output_display_img.shape[0] - 20))

                    if not interface['enabled'] or not interface['client']:
                        mp_drawing_sol.draw_landmarks(output_display_img, hand_landmarks_mp,
                                                      mp_hands_sol.HAND_CONNECTIONS,
                                                      mp_drawing_styles_sol.get_default_hand_landmarks_style(),
                                                      mp_drawing_styles_sol.get_default_hand_connections_style())
                        draw_text_with_outline(output_display_img, f"{hand_label_detected} (Not Controlled)",
                                               hand_label_pos, FONT_FACE, FONT_SCALE_MEDIUM, STATUS_WARN_COLOR,
                                               OUTLINE_COLOR, TEXT_THICKNESS)
                    else:
                        current_hand_frame_info['status'] = "Controlled"
                        landmark_draw_color = STATUS_OK_COLOR if hand_label_detected == "Right" else (
                        180, 180, 255)  # R:Green, L:Pinkish
                        mp_drawing_sol.draw_landmarks(output_display_img, hand_landmarks_mp,
                                                      mp_hands_sol.HAND_CONNECTIONS,
                                                      mp_drawing_sol.DrawingSpec(color=landmark_draw_color, thickness=2,
                                                                                 circle_radius=3),
                                                      mp_drawing_sol.DrawingSpec(color=landmark_draw_color, thickness=1,
                                                                                 circle_radius=1))
                        draw_text_with_outline(output_display_img, f"{hand_label_detected} Hand", hand_label_pos,
                                               FONT_FACE, FONT_SCALE_MEDIUM, landmark_draw_color, OUTLINE_COLOR,
                                               TEXT_THICKNESS)

                        # Pass the hand-specific visual map to get_hand_visual_angles
                        visual_angles = get_hand_visual_angles(hand_landmarks_mp, current_hand_visual_map)
                        current_hand_frame_info.update(visual_angles)

                        ohand_target_cmds = {}
                        hand_ohand_cfg = interface['config']  # Contains OHand output ranges AND visual_input_mapping

                        proc_bends_list = visual_angles['proc_bends']
                        if len(proc_bends_list) == 5:
                            # Use hand-specific visual input ranges for translate_angle
                            vis_map = hand_ohand_cfg.get('visual_input_mapping', {})
                            vis_thumb_b_min = vis_map.get('thumb_bend_input_min', 0);
                            vis_thumb_b_max = vis_map.get('thumb_bend_input_max', 65)
                            ohand_target_cmds[FINGER_THUMB_BEND] = translate_angle(proc_bends_list[0], vis_thumb_b_min,
                                                                                   vis_thumb_b_max,
                                                                                   hand_ohand_cfg['thumb_bend_range'][
                                                                                       0],
                                                                                   hand_ohand_cfg['thumb_bend_range'][
                                                                                       1])

                            vis_other_b_min = vis_map.get('other_fingers_bend_input_min', 0);
                            vis_other_b_max = vis_map.get('other_fingers_bend_input_max', 65)
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
                        vis_map_rot = hand_ohand_cfg.get('visual_input_mapping',
                                                         {})  # Re-get for clarity or ensure vis_map is used
                        vis_thumb_r_min = vis_map_rot.get('thumb_rot_input_min', -90);
                        vis_thumb_r_max = vis_map_rot.get('thumb_rot_input_max', 90)
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

            if commands_to_send_this_frame:  # Minimal console output
                # print(f"FPS: {fps:.1f} - Sending {len(commands_to_send_this_frame)} batch(es).") # Example less verbose
                for item_to_send in commands_to_send_this_frame:
                    hand_lbl, cmds, client_instance = item_to_send['label'], item_to_send['commands'], \
                    OHAND_INTERFACES[item_to_send['label']]['client']
                    for finger_idx_const, angle_val_cmd in cmds.items():
                        try:
                            client_instance.set_finger_target_angle(finger_idx_const, angle_val_cmd)
                        except Exception as e:
                            print(f"    Error setting {hand_lbl} finger {finger_idx_const}: {e}")

            cv2.imshow('Dual OHand Visual Control with Info', output_display_img)
            if cv2.waitKey(5) & 0xFF == ord('q'): print("Quit command received."); break

    except KeyboardInterrupt:
        print("User interrupted (Ctrl+C).")
    except Exception as e:
        print(f"Unexpected error in main loop: {e}"); import traceback; traceback.print_exc()
    finally:
        print("Cleaning up...")
        if camera:
            try:
                camera.stop_stream()
                print("Cam stream stopped.")
            except Exception as e: (
                print(f"Error stopping cam: {e}"))
    cv2.destroyAllWindows();
    print("CV windows closed.")
    for hand_label_cleanup in ['Left', 'Right']:
        interface_cleanup = OHAND_INTERFACES[hand_label_cleanup]
        if interface_cleanup['client'] and interface_cleanup['client'].is_connected:
            print(f"Positioning & disconnecting {hand_label_cleanup} OHand...")
            try:
                cfg_cl = interface_cleanup['config'];
                client_cl = interface_cleanup['client']
                s_o = (cfg_cl.get('finger_bend_range', [0, 0])[0] + cfg_cl.get('finger_bend_range', [0, 0])[1]) / 15
                m_r = (cfg_cl.get('thumb_rot_range', [-45, 45])[0] + cfg_cl.get('thumb_rot_range', [-45, 45])[1]) / 2
                client_cl.set_finger_target_angle(FINGER_THUMB_BEND, s_o);
                client_cl.set_finger_target_angle(FINGER_INDEX, s_o)
                client_cl.set_finger_target_angle(FINGER_MIDDLE, s_o);
                client_cl.set_finger_target_angle(FINGER_RING, s_o)
                client_cl.set_finger_target_angle(FINGER_PINKY, s_o);
                client_cl.set_finger_target_angle(FINGER_THUMB_ROT, m_r)
                time.sleep(0.2)  # Shorter sleep
            except Exception as e_p:
                print(f"  Error safe positioning {hand_label_cleanup}: {e_p}")
            finally:
                try:
                    interface_cleanup['client'].disconnect(); print(f"  {hand_label_cleanup} OHand disconnected.")
                except Exception as e_d:
                    print(f"  Error disconnecting {hand_label_cleanup}: {e_d}")
    print("Cleanup complete.")


if __name__ == '__main__':
    main()