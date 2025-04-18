# Filename: dual_ohand_visual_control_angle_en.py
# Description: Use a single Orbbec camera to detect left/right hands,
#              control two OHand grippers based on calibrated angles loaded from JSON,
#              with English text display using cv2.putText().

import cv2
import mediapipe as mp
import numpy as np
import math
import time
import traceback
import json # To load calibration data
# Removed PIL imports

# --- Import Orbbec Camera Module ---
try:
    # Assume orbbec_camera.py is in camera subdirectory or Python path
    from camera.orbbec_camera import OrbbecCamera, get_serial_numbers
    print("Successfully imported 'orbbec_camera' module.")
except ImportError as e:
    print(f"Error: Failed to import 'orbbec_camera' module. Import Error: {e}")
    exit(1)
except Exception as e:
    print(f"Unknown error during 'orbbec_camera' import: {e}")
    exit(1)

# --- Import OHand Modbus Client Module ---
try:
    # Assume ohand_modbus_client.py (and its dependency roh_registers_v1.py) are accessible
    from ohand_modbus_client import (OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX,
                                     FINGER_MIDDLE, FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT)
    print("Successfully imported 'ohand_modbus_client' module.")
    print("Note: Ensure 'roh_registers_v1.py' is available for 'ohand_modbus_client.py'.")
except ImportError as e:
    print(f"Error: Failed to import 'ohand_modbus_client' or its dependencies. Import Error: {e}")
    exit(1)
except Exception as e:
    print(f"Unknown error during 'ohand_modbus_client' import: {e}")
    exit(1)


# --- Constants and Configuration ---

# Orbbec Camera Config
ORBBEC_CAMERA_SN = None # Use first found device, or specify serial number

# OHand Modbus Config (*** Modify according to your setup ***)
OHAND_LEFT_PORT = '/dev/ttyUSB0'     # *** Serial port for Left OHand ***
OHAND_LEFT_ID = 2                   # *** Modbus Slave ID for Left OHand ***
OHAND_RIGHT_PORT = '/dev/ttyUSB1'    # *** Serial port for Right OHand ***
OHAND_RIGHT_ID = 2                  # *** Modbus Slave ID for Right OHand (MUST BE DIFFERENT from Left) ***
OHAND_BAUDRATE = 115200             # Modbus Baudrate for OHand

# OHand Logical Position Range
OHAND_POS_OPEN = 0       # Logical position value when fingers are fully open
OHAND_POS_CLOSED = 65535  # Logical position value when fingers are fully closed
OHAND_THUMB_ROT_MIN = 65535     # Logical position for one limit of thumb rotation
OHAND_THUMB_ROT_MAX = 0 # Logical position for the other limit of thumb rotation

# --- Default Input Angle Range Constants (Fallback if JSON loading fails) ---
DEFAULT_ANGLE_RANGES = {
    "BEND_ANGLE_STRAIGHT": 0.0,    # Bend metric when fingers are straight (0=straight)
    "BEND_ANGLE_BENT": 300.0,      # Bend metric when fingers are bent (calibrate!)
    "THUMB_BEND_ANGLE_STRAIGHT": 0.0,
    "THUMB_BEND_ANGLE_BENT": 280.0,  # (calibrate!)
    "THUMB_ROT_ANGLE_MIN": -90.0, # Thumb rotation angle range (calibrate!)
    "THUMB_ROT_ANGLE_MAX": 30.0,   # (calibrate!)
}

# Calibration Filename
CALIBRATION_FILE = 'hand_angle_calibration.json'

# Other Constants
SEND_INTERVAL = 0.05 # Minimum interval between sending commands (seconds)
POS_CHANGE_THRESHOLD = 500 # Send command if OHand position changes by more than this threshold
SMOOTHING_FACTOR = 0.4    # Smoothing factor for EMA (0 < alpha < 1)
ESC_KEY = 27              # OpenCV key code for quitting

# OpenCV Font Settings
FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.7
FONT_THICKNESS = 2

# --- MediaPipe Hands Initialization ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    model_complexity=1,
    max_num_hands=2,
    min_detection_confidence=0.75,
    min_tracking_confidence=0.75)
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# --- OHand Modbus Client Initialization ---
ohand_left_client: OHandModbusClient | None = None
ohand_right_client: OHandModbusClient | None = None
try:
    print(f"Initializing Left OHand Modbus (Port: {OHAND_LEFT_PORT}, ID: {OHAND_LEFT_ID})")
    ohand_left_client = OHandModbusClient(port=OHAND_LEFT_PORT, slave_id=OHAND_LEFT_ID, baudrate=OHAND_BAUDRATE, timeout=1.5)
except Exception as e:
    print(f"Failed to create Left OHandModbusClient instance: {e}")
try:
    if OHAND_LEFT_ID == OHAND_RIGHT_ID: # Check if IDs are identical (problematic)
         print(f"Warning: Left and Right OHand Slave IDs ({OHAND_LEFT_ID}) are the same! Please assign unique IDs.")
    print(f"Initializing Right OHand Modbus (Port: {OHAND_RIGHT_PORT}, ID: {OHAND_RIGHT_ID})")
    ohand_right_client = OHandModbusClient(port=OHAND_RIGHT_PORT, slave_id=OHAND_RIGHT_ID, baudrate=OHAND_BAUDRATE, timeout=1.5)
except Exception as e:
    print(f"Failed to create Right OHandModbusClient instance: {e}")


# --- Utility Functions ---
def map_value(value, in_min, in_max, out_min, out_max):
    """Linearly maps a value from one range to another, clamping to the output range."""
    if in_max == in_min: return out_min
    # Clamp input value
    value = max(min(value, in_max), in_min)
    # Map value
    mapped = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    # Clamp output value
    if out_min <= out_max: return max(min(mapped, out_max), out_min)
    else: return max(min(mapped, out_min), out_max)

def calculate_angle_3d(p1, p2, p3):
    """Calculates the 3D angle (degrees) at p2 formed by vectors p1->p2 and p3->p2."""
    if p1 is None or p2 is None or p3 is None: return 0.0
    v1 = np.array([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z]); v2 = np.array([p3.x - p2.x, p3.y - p2.y, p3.z - p2.z])
    len_v1 = np.linalg.norm(v1); len_v2 = np.linalg.norm(v2)
    if len_v1 == 0 or len_v2 == 0: return 0.0
    v1_u = v1 / len_v1; v2_u = v2 / len_v2
    dot_product = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    return np.degrees(np.arccos(dot_product))

def calculate_thumb_rotation_xy_angle(wrist, thumb_mcp, index_mcp):
     """Calculates the angle (degrees) between WRIST->THUMB_MCP and WRIST->INDEX_FINGER_MCP vectors in the XY plane."""
     if wrist is None or thumb_mcp is None or index_mcp is None: return 0.0
     v_thumb = np.array([thumb_mcp.x - wrist.x, thumb_mcp.y - wrist.y]); v_index = np.array([index_mcp.x - wrist.x, index_mcp.y - wrist.y])
     if np.linalg.norm(v_thumb) < 1e-6 or np.linalg.norm(v_index) < 1e-6: return 0.0
     angle_thumb = math.atan2(v_thumb[1], v_thumb[0]); angle_index = math.atan2(v_index[1], v_index[0])
     angle_diff_rad = angle_thumb - angle_index; angle_diff_deg = np.degrees(angle_diff_rad)
     return (angle_diff_deg + 180) % 360 - 180 # Normalize to -180 to 180

# --- OHand Control Function ---
def send_to_ohand_individual(client: OHandModbusClient | None, target_positions: list):
    """Sends target position commands to the 6 DoF of the specified OHand client."""
    if not (client and client.is_connected): return
    if len(target_positions) != 6: print(f"Error: target_positions list length must be 6"); return
    finger_indices = [FINGER_THUMB_BEND, FINGER_INDEX, FINGER_MIDDLE, FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT]
    try:
        for i, finger_index in enumerate(finger_indices):
            target_pos_int = int(round(target_positions[i]))
            target_pos_int = max(0, min(target_pos_int, 65535)) # Clamp value
            client.set_finger_target_pos(finger_index, target_pos_int)
    except Exception as e: print(f"Error sending OHand command (Client: {client.client.port if client else 'N/A'}): {e}")

# --- Load Calibration Data Function ---
def load_calibrated_ranges(filename, defaults):
    """Loads calibration ranges from JSON, validates, and returns usable constants."""
    print(f"Attempting to load calibration data from '{filename}'...")
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            data_loaded = json.load(f)
        print("JSON file loaded successfully. Parsing ranges...")

        final_ranges = defaults.copy() # Start with defaults

        def get_val(key, type):
            default_inf = float('inf') if type == 'min' else float('-inf')
            val = data_loaded.get(key, {}).get(type)
            return default_inf if val is None or not isinstance(val, (int, float)) else float(val)

        # Extract individual min/max
        bend_mins = []; bend_maxs = []
        for bend_key in ["Index Bend", "Middle Bend", "Ring Bend", "Pinky Bend"]:
             min_val = get_val(bend_key, "min"); max_val = get_val(bend_key, "max")
             if min_val < max_val : bend_mins.append(min_val); bend_maxs.append(max_val)
        thumb_bend_min = get_val("Thumb Bend", "min"); thumb_bend_max = get_val("Thumb Bend", "max")
        thumb_rot_min = get_val("Thumb Rotation", "min"); thumb_rot_max = get_val("Thumb Rotation", "max")

        # Calculate combined ranges, using defaults if necessary
        bend_straight = np.nanmin(bend_mins) if bend_mins else defaults["BEND_ANGLE_STRAIGHT"]
        bend_bent = np.nanmax(bend_maxs) if bend_maxs else defaults["BEND_ANGLE_BENT"]
        if bend_straight < bend_bent:
            final_ranges["BEND_ANGLE_STRAIGHT"] = bend_straight
            final_ranges["BEND_ANGLE_BENT"] = bend_bent
            print(f"  Loaded Bend range (Combined): {bend_straight:.2f} - {bend_bent:.2f}")
        else: print(f"  Warning: Loaded combined Bend range invalid. Using defaults.")

        if thumb_bend_min < thumb_bend_max:
             final_ranges["THUMB_BEND_ANGLE_STRAIGHT"] = thumb_bend_min
             final_ranges["THUMB_BEND_ANGLE_BENT"] = thumb_bend_max
             print(f"  Loaded Thumb Bend range: {thumb_bend_min:.2f} - {thumb_bend_max:.2f}")
        else: print(f"  Warning: Loaded Thumb Bend range invalid. Using defaults.")

        if thumb_rot_min < thumb_rot_max:
             final_ranges["THUMB_ROT_ANGLE_MIN"] = thumb_rot_min
             final_ranges["THUMB_ROT_ANGLE_MAX"] = thumb_rot_max
             print(f"  Loaded Thumb Rot range: {thumb_rot_min:.2f} - {thumb_rot_max:.2f}")
        else: print(f"  Warning: Loaded Thumb Rot range invalid. Using defaults.")
        return final_ranges

    except FileNotFoundError: print(f"Calibration file '{filename}' not found. Using default angle ranges."); return defaults
    except (json.JSONDecodeError, TypeError, KeyError) as e: print(f"Error parsing calibration file {filename}: {e}. Using default angle ranges."); return defaults
    except Exception as e: print(f"Unknown error loading JSON: {e}. Using default angle ranges."); return defaults

# --- Main Execution Logic ---
def main():
    # --- Load Calibrated Ranges or Use Defaults ---
    active_angle_ranges = load_calibrated_ranges(CALIBRATION_FILE, DEFAULT_ANGLE_RANGES)
    BEND_ANGLE_STRAIGHT = active_angle_ranges["BEND_ANGLE_STRAIGHT"]
    BEND_ANGLE_BENT = active_angle_ranges["BEND_ANGLE_BENT"]
    THUMB_BEND_ANGLE_STRAIGHT = active_angle_ranges["THUMB_BEND_ANGLE_STRAIGHT"]
    THUMB_BEND_ANGLE_BENT = active_angle_ranges["THUMB_BEND_ANGLE_BENT"]
    THUMB_ROT_ANGLE_MIN = active_angle_ranges["THUMB_ROT_ANGLE_MIN"]
    THUMB_ROT_ANGLE_MAX = active_angle_ranges["THUMB_ROT_ANGLE_MAX"]

    print("\n--- Final Angle Ranges Being Used ---")
    print(f"Bend Straight (Combined): {BEND_ANGLE_STRAIGHT:.2f}")
    print(f"Bend Bent (Combined):     {BEND_ANGLE_BENT:.2f}")
    print(f"Thumb Bend Straight: {THUMB_BEND_ANGLE_STRAIGHT:.2f}")
    print(f"Thumb Bend Bent:     {THUMB_BEND_ANGLE_BENT:.2f}")
    print(f"Thumb Rot Min:       {THUMB_ROT_ANGLE_MIN:.2f}")
    print(f"Thumb Rot Max:       {THUMB_ROT_ANGLE_MAX:.2f}")
    print("-----------------------------------\n")

    # --- Initialize Devices ---
    if ohand_left_client is None and ohand_right_client is None: print("Error: Both OHand clients failed to initialize. Exiting."); return
    orbbec_camera = None; ohand_left_connected = False; ohand_right_connected = False
    try:
        # Init Orbbec Camera
        print("Finding Orbbec devices...")
        available_sns = get_serial_numbers();
        if not available_sns: print("Error: No Orbbec devices found."); return
        camera_sn = ORBBEC_CAMERA_SN if ORBBEC_CAMERA_SN else available_sns[0]
        print(f"Initializing Orbbec camera (SN: {camera_sn if camera_sn else 'First available'})...")
        orbbec_camera = OrbbecCamera(camera_sn if camera_sn else available_sns[0])
        print(f"Orbbec camera {orbbec_camera.get_serial_number()} initialized.")

        # Connect OHand Devices
        if ohand_left_client:
            print(f"Connecting Left OHand ({OHAND_LEFT_PORT})...")
            if ohand_left_client.connect(): ohand_left_connected = True; print("Left OHand connected.")
            else: print(f"Error: Failed to connect Left OHand ({OHAND_LEFT_PORT}).")
        if ohand_right_client:
            print(f"Connecting Right OHand ({OHAND_RIGHT_PORT})...")
            if ohand_right_client.connect(): ohand_right_connected = True; print("Right OHand connected.")
            else: print(f"Error: Failed to connect Right OHand ({OHAND_RIGHT_PORT}).")
        if not (ohand_left_connected or ohand_right_connected): print("Warning: No OHand grippers connected. Visual tracking only.")

        # Start Orbbec Stream
        print("Starting Orbbec stream (Color only)...")
        orbbec_camera.start_stream(color_stream=True, depth_stream=True)
        print("Orbbec stream started.")

        # --- Loop Variables ---
        last_send_time = time.time()
        smoothed_positions_left = [OHAND_POS_OPEN] * 6
        smoothed_positions_right = [OHAND_POS_OPEN] * 6
        last_sent_positions_left = smoothed_positions_left[:]
        last_sent_positions_right = smoothed_positions_right[:]
        frame_counter = 0
        print("Entering main loop (Press 'q' in OpenCV window to quit)...")

        # --- Main Loop ---
        while True:
            frame_counter += 1
            # 1. Get Image
            color_image_orig, _, _ = orbbec_camera.get_frames()
            if color_image_orig is None: time.sleep(0.01); continue
            if not isinstance(color_image_orig, np.ndarray): continue
            color_image = color_image_orig.copy()

            # 2. MediaPipe Processing
            image_height, image_width, _ = color_image.shape
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            results = hands.process(image_rgb)
            color_image.flags.writeable = True

            # Reset states for this frame
            raw_target_positions_left = last_sent_positions_left[:]
            raw_target_positions_right = last_sent_positions_right[:]
            detected_left_this_frame = False
            detected_right_this_frame = False

            # 3. Extract Angles & Map to OHand Positions
            if results.multi_hand_landmarks and results.multi_handedness:
                for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    handedness = results.multi_handedness[i].classification[0].label
                    score = results.multi_handedness[i].classification[0].score

                    mp_drawing.draw_landmarks(color_image, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS,
                                              mp_drawing_styles.get_default_hand_landmarks_style(),
                                              mp_drawing_styles.get_default_hand_connections_style())

                    # Determine display label and color based on CONTROL target
                    display_label = "Ctrl: Right Claw" if handedness == 'Left' else "Ctrl: Left Claw"
                    display_color = (0, 255, 0) if handedness == 'Left' else (255, 0, 255) # Green=Right, Magenta=Left
                    label_pos = (int(hand_landmarks.landmark[0].x * image_width) - 10, int(hand_landmarks.landmark[0].y * image_height) - 20)
                    cv2.putText(color_image, f"{display_label} ({score:.2f})", label_pos, FONT_FACE, FONT_SCALE, display_color, FONT_THICKNESS)


                    landmarks = hand_landmarks.landmark
                    target_hand_raw_positions = [OHAND_POS_OPEN] * 6

                    try:
                        # Calculate bend metrics (0=straight, higher=bent)
                        bend_metrics = {}
                        bend_metrics["Index"] = (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP], landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP], landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_DIP])) + \
                                              (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP], landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_DIP], landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP]))
                        bend_metrics["Middle"] = (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP], landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP], landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_DIP])) + \
                                               (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP], landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_DIP], landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP]))
                        bend_metrics["Ring"] = (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_MCP], landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_PIP], landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_DIP])) + \
                                             (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_PIP], landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_DIP], landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_TIP]))
                        bend_metrics["Pinky"] = (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.PINKY_MCP], landmarks[mp.solutions.hands.HandLandmark.PINKY_PIP], landmarks[mp.solutions.hands.HandLandmark.PINKY_DIP])) + \
                                              (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.PINKY_PIP], landmarks[mp.solutions.hands.HandLandmark.PINKY_DIP], landmarks[mp.solutions.hands.HandLandmark.PINKY_TIP]))
                        bend_metrics["Thumb"] = (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.THUMB_CMC], landmarks[mp.solutions.hands.HandLandmark.THUMB_MCP], landmarks[mp.solutions.hands.HandLandmark.THUMB_IP])) + \
                                              (180.0-calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.THUMB_MCP], landmarks[mp.solutions.hands.HandLandmark.THUMB_IP], landmarks[mp.solutions.hands.HandLandmark.THUMB_TIP]))

                        # Calculate thumb rotation angle
                        thumb_rot_angle = calculate_thumb_rotation_xy_angle(landmarks[mp.solutions.hands.HandLandmark.WRIST],
                                                                            landmarks[mp.solutions.hands.HandLandmark.THUMB_MCP],
                                                                            landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP])

                        # Map angles to OHand positions using loaded/default ranges
                        target_hand_raw_positions[FINGER_INDEX] = map_value(bend_metrics["Index"], BEND_ANGLE_STRAIGHT, BEND_ANGLE_BENT, OHAND_POS_OPEN, OHAND_POS_CLOSED)
                        target_hand_raw_positions[FINGER_MIDDLE]= map_value(bend_metrics["Middle"],BEND_ANGLE_STRAIGHT, BEND_ANGLE_BENT, OHAND_POS_OPEN, OHAND_POS_CLOSED)
                        target_hand_raw_positions[FINGER_RING]  = map_value(bend_metrics["Ring"],  BEND_ANGLE_STRAIGHT, BEND_ANGLE_BENT, OHAND_POS_OPEN, OHAND_POS_CLOSED)
                        target_hand_raw_positions[FINGER_PINKY] = map_value(bend_metrics["Pinky"], BEND_ANGLE_STRAIGHT, BEND_ANGLE_BENT, OHAND_POS_OPEN, OHAND_POS_CLOSED)
                        target_hand_raw_positions[FINGER_THUMB_BEND] = map_value(bend_metrics["Thumb"], THUMB_BEND_ANGLE_STRAIGHT, THUMB_BEND_ANGLE_BENT, OHAND_POS_OPEN, OHAND_POS_CLOSED)
                        target_hand_raw_positions[FINGER_THUMB_ROT] = map_value(thumb_rot_angle, THUMB_ROT_ANGLE_MIN, THUMB_ROT_ANGLE_MAX, OHAND_THUMB_ROT_MIN, OHAND_THUMB_ROT_MAX)

                    except Exception as calc_e: print(f"Error calculating/mapping angles: {calc_e}")

                    # Update raw targets based on handedness (swapped logic)
                    if handedness == 'Left': raw_target_positions_right = target_hand_raw_positions[:]; detected_right_this_frame = True
                    elif handedness == 'Right': raw_target_positions_left = target_hand_raw_positions[:]; detected_left_this_frame = True

            # 3a. Apply EMA Smoothing
            for i in range(6):
                if detected_left_this_frame: smoothed_positions_left[i] = SMOOTHING_FACTOR * raw_target_positions_left[i] + (1 - SMOOTHING_FACTOR) * smoothed_positions_left[i]
                if detected_right_this_frame: smoothed_positions_right[i] = SMOOTHING_FACTOR * raw_target_positions_right[i] + (1 - SMOOTHING_FACTOR) * smoothed_positions_right[i]

            # 4. Send Commands to OHand (using smoothed values)
            current_time = time.time()
            should_send = (current_time - last_send_time) > SEND_INTERVAL
            left_pos_changed = False
            if detected_left_this_frame:
                for i in range(6):
                    if abs(round(smoothed_positions_left[i]) - round(last_sent_positions_left[i])) > POS_CHANGE_THRESHOLD: left_pos_changed = True; break
            right_pos_changed = False
            if detected_right_this_frame:
                 for i in range(6):
                    if abs(round(smoothed_positions_right[i]) - round(last_sent_positions_right[i])) > POS_CHANGE_THRESHOLD: right_pos_changed = True; break

            send_triggered = False
            if should_send or left_pos_changed or right_pos_changed:
                if ohand_left_connected and detected_left_this_frame and left_pos_changed:
                    send_to_ohand_individual(ohand_left_client, smoothed_positions_left)
                    last_sent_positions_left = smoothed_positions_left[:]; send_triggered = True
                if ohand_right_connected and detected_right_this_frame and right_pos_changed:
                    send_to_ohand_individual(ohand_right_client, smoothed_positions_right)
                    last_sent_positions_right = smoothed_positions_right[:]; send_triggered = True
                # Send based on time interval even if no large change detected
                if should_send and not send_triggered:
                     if ohand_left_connected and detected_left_this_frame: send_to_ohand_individual(ohand_left_client, smoothed_positions_left); last_sent_positions_left = smoothed_positions_left[:]
                     if ohand_right_connected and detected_right_this_frame: send_to_ohand_individual(ohand_right_client, smoothed_positions_right); last_sent_positions_right = smoothed_positions_right[:]
                     send_triggered = True # Mark as sent even if no change threshold met
                if send_triggered: last_send_time = current_time

            # 5. Display Image
            # Optional: Add text display for smoothed OHand target values
            # cv2.putText(color_image, f"LPos:{[int(p) for p in smoothed_positions_left]}", (10, image_height - 30), FONT_FACE, 0.4, (255, 0, 255), 1)
            # cv2.putText(color_image, f"RPos:{[int(p) for p in smoothed_positions_right]}", (10, image_height - 10), FONT_FACE, 0.4, (0, 255, 0), 1)
            cv2.imshow('Orbbec Dual Hand OHand Control (Calibrated + EN)', color_image) # Updated title

            # 6. Handle Quit Key
            key = cv2.waitKey(5) & 0xFF
            if key == ord('q') or key == ESC_KEY: print("Quit key pressed."); break

    # --- Exception Handling & Cleanup ---
    except KeyboardInterrupt: print("\nCtrl+C detected. Exiting...")
    except Exception as e: print(f"\nUnhandled error in main loop: {e}"); traceback.print_exc()
    finally:
        print("Cleaning up resources...")
        if orbbec_camera is not None:
            print("Stopping Orbbec pipeline...")
            try: orbbec_camera.stop()
            except Exception as e: print(f"Error stopping Orbbec camera: {e}")
        if ohand_left_client is not None:
            print("Disconnecting Left OHand...")
            try: ohand_left_client.disconnect()
            except Exception as e: print(f"Error disconnecting Left OHand: {e}")
        if ohand_right_client is not None:
            print("Disconnecting Right OHand...")
            try: ohand_right_client.disconnect()
            except Exception as e: print(f"Error disconnecting Right OHand: {e}")
        cv2.destroyAllWindows()
        print("OpenCV windows closed.")
        print("Program finished.")

# --- Program Entry Point ---
if __name__ == "__main__":
    main()