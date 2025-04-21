# Filename: calibrate_hand_angles_en.py
# Description: Calibrate MediaPipe hand angle ranges by capturing user's limit poses,
#              using standard OpenCV text display, and saving results to JSON.
'''
执行校准动作 (核心步骤):

放置单手: 将你的一只手（左手或右手均可，建议一次校准专注于一只手）清晰地放入摄像头视野中央，确保 MediaPipe 能稳定检测到关键点（手上会绘制绿色的点和线）。
观察数值: 注意观察屏幕上 "Current Values" 的变化，了解不同姿势对应的角度值。同时观察 "Recorded Range" 如何根据你的动作更新 Min/Max 记录。
执行极限姿势 (每个姿势保持几秒):
'o' (Open/Straight - 完全伸直): 将手掌和所有手指完全伸直并张开，保持这个姿势几秒钟。这样做是为了让程序记录下各个弯曲度量 (Bend 相关) 的最小值 (接近 0)，以及拇指旋转 (Thumb Rotation) 在该姿势下的角度值（可能是 Min 或 Max 的一端）。按 'o' 键时终端会打印提示，但记录是自动的。
'c' (Closed/Fist - 完全握拳): 将手紧紧握成拳头，保持几秒钟。这样做是为了记录下各个弯曲度量 (Bend 相关) 的最大值。
'i' (Thumb In/Adducted - 拇指内收): 将拇指尽可能向内贴近手掌或向小指方向移动，保持几秒钟。这会记录下 Thumb Rotation 角度的一个极值 (可能是 Min)。
't' (Thumb Out/Abducted - 拇指外展): 将拇指尽可能向外远离手掌张开（类似点赞手势但更向外），保持几秒钟。这会记录下 Thumb Rotation 角度的另一个极值 (可能是 Max)。
重复与微调: 反复执行上述极限姿势，可以在视野内稍微移动手的位置或角度，确保记录到的 Min / Max 值能够覆盖你自然的运动范围，并且数值趋于稳定。
辅助操作:

'r' (Reset - 重置): 如果在校准过程中觉得记录的范围不准确，可以随时按下 'r' 键。所有记录的 Min / Max 值将恢复到初始状态（最小值变无穷大，最大值变负无穷大），你可以重新开始执行极限姿势。终端会打印 "Resetting calibration ranges..."。
's' (Save & Print - 保存并打印): 在校准过程中，可以随时按 's' 键。程序会将当前记录的范围保存到 hand_angle_calibration.json 文件中，并在终端打印出这些范围。这方便你中途查看结果或备份。
结束校准:

当你对屏幕右侧显示的 "Recorded Range (Min / Max)" 感到满意，认为它准确反映了你手部的活动范围时，按下 'q' 键。
'''
import cv2
import mediapipe as mp
import numpy as np
import math
import time
import traceback
import json  # For saving/loading calibration data

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

# --- Constants and Configuration ---
ORBBEC_CAMERA_SN = None  # Use first found device
CALIBRATION_FILE = 'hand_angle_calibration.json'  # Filename to save calibration data

# MediaPipe Configuration
MODEL_COMPLEXITY = 1
MIN_DETECTION_CONFIDENCE = 0.7
MIN_TRACKING_CONFIDENCE = 0.7
MAX_NUM_HANDS = 1  # Calibrate one hand at a time

# Angle Metric Names (Internal keys for the dictionary)
INTERNAL_METRIC_KEYS = [
    "Index Bend", "Middle Bend", "Ring Bend", "Pinky Bend", "Thumb Bend",
    "Thumb Rotation"
]

# OpenCV Font Settings
FONT_FACE = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE_INSTR = 0.6  # Font scale for instructions
FONT_SCALE_VALUE = 0.5  # Font scale for values
FONT_THICKNESS = 1


# --- Angle Calculation Functions (Keep as before) ---
def calculate_angle_3d(p1, p2, p3):
    """Calculates angle (in degrees) at p2 formed by vectors p1->p2 and p3->p2."""
    if p1 is None or p2 is None or p3 is None: return 0.0
    v1 = np.array([p1.x - p2.x, p1.y - p2.y, p1.z - p2.z])
    v2 = np.array([p3.x - p2.x, p3.y - p2.y, p3.z - p2.z])
    len_v1 = np.linalg.norm(v1)
    len_v2 = np.linalg.norm(v2)
    if len_v1 == 0 or len_v2 == 0: return 0.0
    v1_u = v1 / len_v1
    v2_u = v2 / len_v2
    dot_product = np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)
    angle_rad = np.arccos(dot_product)
    return np.degrees(angle_rad)


def calculate_thumb_rotation_xy_angle(wrist, thumb_mcp, index_mcp):
    """Calculates the angle (degrees) between WRIST->THUMB_MCP and WRIST->INDEX_FINGER_MCP vectors in the XY plane."""
    if wrist is None or thumb_mcp is None or index_mcp is None: return 0.0
    v_thumb = np.array([thumb_mcp.x - wrist.x, thumb_mcp.y - wrist.y])
    v_index = np.array([index_mcp.x - wrist.x, index_mcp.y - wrist.y])
    if np.linalg.norm(v_thumb) < 1e-6 or np.linalg.norm(v_index) < 1e-6: return 0.0
    angle_thumb = math.atan2(v_thumb[1], v_thumb[0])
    angle_index = math.atan2(v_index[1], v_index[0])
    angle_diff_rad = angle_thumb - angle_index
    angle_diff_deg = np.degrees(angle_diff_rad)
    angle_diff_deg = (angle_diff_deg + 180) % 360 - 180  # Normalize to -180 to 180
    return angle_diff_deg


# --- Initialize/Load/Save Calibration Ranges (Keep JSON functions as before) ---
def initialize_calibration_ranges():
    """Initializes dictionary to store min/max values"""
    ranges = {}
    for key in INTERNAL_METRIC_KEYS:
        ranges[key] = {"min": float('inf'), "max": float('-inf')}
    return ranges


def save_calibration_to_json(data, filename):
    """Saves calibration data to JSON, handling infinity."""
    data_to_save = {}
    for key, value_dict in data.items():
        min_val = value_dict.get('min', float('inf'))
        max_val = value_dict.get('max', float('-inf'))
        data_to_save[key] = {
            "min": None if min_val == float('inf') else round(min_val, 2),
            "max": None if max_val == float('-inf') else round(max_val, 2)
        }
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data_to_save, f, indent=4, ensure_ascii=False)
        print(f"Calibration data saved to: {filename}")
        return True
    except Exception as e:
        print(f"Error saving calibration JSON {filename}: {e}")
        return False


def load_calibration_from_json(filename):
    """Loads calibration data from JSON, handling None."""
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            data_loaded = json.load(f)
        loaded_ranges = initialize_calibration_ranges()
        for key in loaded_ranges:
            if key in data_loaded and isinstance(data_loaded[key], dict):
                min_val = data_loaded[key].get('min')
                max_val = data_loaded[key].get('max')
                loaded_ranges[key]['min'] = float('inf') if min_val is None else float(min_val)
                loaded_ranges[key]['max'] = float('-inf') if max_val is None else float(max_val)
            # else: # No warning needed if key missing, just use default inf/-inf
        print(f"Successfully loaded previous calibration data from {filename}.")
        return loaded_ranges
    except FileNotFoundError:
        print(f"Previous calibration file '{filename}' not found. Starting with defaults.")
        return initialize_calibration_ranges()
    except Exception as e:
        print(f"Error loading calibration JSON {filename}: {e}. Starting with defaults.")
        return initialize_calibration_ranges()


def print_calibration_results(calibration_ranges):
    """Formats and prints calibration results to console."""
    print("\n--- Calibration Ranges Recorded (for main script constants) ---")
    print(
        "# NOTE: 'Bend Angle Straight/Bent' maps to bend metric (0=straight), 'Thumb Rot Min/Max' maps to rotation angle")

    bend_keys = ["Index Bend", "Middle Bend", "Ring Bend", "Pinky Bend"]
    valid_mins = [calibration_ranges[k]['min'] for k in bend_keys if calibration_ranges[k]['min'] != float('inf')]
    valid_maxs = [calibration_ranges[k]['max'] for k in bend_keys if calibration_ranges[k]['max'] != float('-inf')]
    bend_straight = np.nanmin(valid_mins) if valid_mins else 0.0
    bend_bent = np.nanmax(valid_maxs) if valid_maxs else 300.0

    thumb_bend_straight = calibration_ranges["Thumb Bend"]['min']
    thumb_bend_bent = calibration_ranges["Thumb Bend"]['max']
    thumb_rot_min = calibration_ranges["Thumb Rotation"]['min']
    thumb_rot_max = calibration_ranges["Thumb Rotation"]['max']

    print(f"BEND_ANGLE_STRAIGHT = {bend_straight:.2f}")
    print(f"BEND_ANGLE_BENT = {bend_bent:.2f}")
    # print(f"THUMB_BEND_ANGLE_STRAIGHT = {thumb_bend_straight:.2f if thumb_bend_straight != float('inf') else '# No valid value recorded, recalibrate or set manually'}")
    # print(f"THUMB_BEND_ANGLE_BENT = {thumb_bend_bent:.2f if thumb_bend_bent != float('-inf') else '# No valid value recorded, recalibrate or set manually'}")
    # print(f"THUMB_ROT_ANGLE_MIN = {thumb_rot_min:.2f if thumb_rot_min != float('inf') else '# No valid value recorded, recalibrate or set manually'}")
    # print(f"THUMB_ROT_ANGLE_MAX = {thumb_rot_max:.2f if thumb_rot_max != float('-inf') else '# No valid value recorded, recalibrate or set manually'}")
    # print("----------------------------------------------------\n")


# --- Main Calibration Function ---
def run_calibration():
    calibration_ranges = load_calibration_from_json(CALIBRATION_FILE)  # Load previous data
    orbbec_camera = None

    try:
        # --- Initialize Orbbec Camera ---
        print("Finding Orbbec devices...")
        available_sns = get_serial_numbers()
        if not available_sns: print("Error: No Orbbec devices found."); return
        camera_sn = ORBBEC_CAMERA_SN if ORBBEC_CAMERA_SN else available_sns[0]
        print(f"Initializing Orbbec camera (SN: {camera_sn if camera_sn else 'First available'})...")
        orbbec_camera = OrbbecCamera(camera_sn if camera_sn else available_sns[0])
        print(f"Orbbec camera {orbbec_camera.get_serial_number()} initialized.")

        # --- Start Orbbec Stream ---
        print("Starting Orbbec stream (Color only)...")
        orbbec_camera.start_stream(color_stream=True, depth_stream=True)
        print("Orbbec stream started.")

        # --- Initialize MediaPipe Hands (Single hand) ---
        hands = mp.solutions.hands.Hands(
            model_complexity=MODEL_COMPLEXITY,
            max_num_hands=MAX_NUM_HANDS,
            min_detection_confidence=MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=MIN_TRACKING_CONFIDENCE
        )
        mp_drawing = mp.solutions.drawing_utils

        # --- Calibration Loop ---
        frame_counter = 0
        print("\n" + "=" * 40)
        print(" Hand Angle Calibration ")
        print(" Place **one hand** in the camera view")
        print(" Keys:")
        print("  'o' - Hold hand fully OPEN/STRAIGHT")
        print("  'c' - Hold hand fully CLOSED/FIST")
        print("  'i' - Hold thumb fully IN/ADDUCTED")
        print("  't' - Hold thumb fully OUT/ABDUCTED")
        print("  'r' - RESET all recorded ranges")
        print("  's' - SAVE and print current ranges")
        print("  'q' - QUIT and save final ranges")
        print("=" * 40 + "\n")

        while True:
            frame_counter += 1
            # 1. Get Image
            color_image_orig, _, _ = orbbec_camera.get_frames()
            if color_image_orig is None: time.sleep(0.01); continue
            if not isinstance(color_image_orig, np.ndarray): continue
            color_image = color_image_orig.copy()  # Work on a copy

            # 2. MediaPipe Processing
            image_height, image_width, _ = color_image.shape
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            results = hands.process(image_rgb)
            color_image.flags.writeable = True

            current_metrics = {key: None for key in INTERNAL_METRIC_KEYS}

            # 3. Calculate Angles & Update Ranges
            if results.multi_hand_landmarks:
                hand_landmarks = results.multi_hand_landmarks[0]
                mp_drawing.draw_landmarks(color_image, hand_landmarks, mp.solutions.hands.HAND_CONNECTIONS)
                landmarks = hand_landmarks.landmark

                try:
                    # Calculate Bend Metrics (0=straight, higher=bent)
                    pip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP],
                                             landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_DIP])
                    dip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_DIP],
                                             landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_TIP])
                    current_metrics["Index Bend"] = (180.0 - pip) + (180.0 - dip) if pip > 0 and dip > 0 else 0
                    # Middle
                    pip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_MCP],
                                             landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_DIP])
                    dip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_DIP],
                                             landmarks[mp.solutions.hands.HandLandmark.MIDDLE_FINGER_TIP])
                    current_metrics["Middle Bend"] = (180.0 - pip) + (180.0 - dip) if pip > 0 and dip > 0 else 0
                    # Ring
                    pip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_MCP],
                                             landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_DIP])
                    dip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_DIP],
                                             landmarks[mp.solutions.hands.HandLandmark.RING_FINGER_TIP])
                    current_metrics["Ring Bend"] = (180.0 - pip) + (180.0 - dip) if pip > 0 and dip > 0 else 0
                    # Pinky
                    pip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.PINKY_MCP],
                                             landmarks[mp.solutions.hands.HandLandmark.PINKY_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.PINKY_DIP])
                    dip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.PINKY_PIP],
                                             landmarks[mp.solutions.hands.HandLandmark.PINKY_DIP],
                                             landmarks[mp.solutions.hands.HandLandmark.PINKY_TIP])
                    current_metrics["Pinky Bend"] = (180.0 - pip) + (180.0 - dip) if pip > 0 and dip > 0 else 0
                    # Thumb Bend
                    mcp = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.THUMB_CMC],
                                             landmarks[mp.solutions.hands.HandLandmark.THUMB_MCP],
                                             landmarks[mp.solutions.hands.HandLandmark.THUMB_IP])
                    ip = calculate_angle_3d(landmarks[mp.solutions.hands.HandLandmark.THUMB_MCP],
                                            landmarks[mp.solutions.hands.HandLandmark.THUMB_IP],
                                            landmarks[mp.solutions.hands.HandLandmark.THUMB_TIP])
                    current_metrics["Thumb Bend"] = (180.0 - mcp) + (180.0 - ip) if mcp > 0 and ip > 0 else 0

                    # Calculate Thumb Rotation Angle
                    current_metrics["Thumb Rotation"] = calculate_thumb_rotation_xy_angle(
                        landmarks[mp.solutions.hands.HandLandmark.WRIST],
                        landmarks[mp.solutions.hands.HandLandmark.THUMB_MCP],
                        landmarks[mp.solutions.hands.HandLandmark.INDEX_FINGER_MCP])

                    # Auto-update recorded Min/Max
                    for name, value in current_metrics.items():
                        if value is not None and name in calibration_ranges:
                            calibration_ranges[name]["min"] = min(calibration_ranges[name]["min"], value)
                            calibration_ranges[name]["max"] = max(calibration_ranges[name]["max"], value)
                except Exception as calc_e:
                    print(f"Error calculating angles: {calc_e}")

            # 4. Display Information using cv2.putText
            info_area_height = 240  # Reduced height needed for text
            if image_width <= 0 or info_area_height <= 0: continue  # Skip if dimensions invalid
            info_area = np.zeros((info_area_height, image_width, 3), dtype=np.uint8)

            # Instructions
            instr_text = "Keys: (o)pen (c)losed (i)n (t)hum_out (r)eset (s)ave (q)uit"
            cv2.putText(info_area, instr_text, (10, 25), FONT_FACE, FONT_SCALE_INSTR, (0, 255, 255), FONT_THICKNESS,
                        cv2.LINE_AA)

            col1_x = 10
            col2_x = 400  # Adjust as needed for your screen width
            y_start = 60
            line_height = 25  # Increased line height for cv2.putText

            cv2.putText(info_area, "Current Values:", (col1_x, y_start), FONT_FACE, FONT_SCALE_INSTR, (255, 255, 0),
                        FONT_THICKNESS, cv2.LINE_AA)  # Cyan title
            cv2.putText(info_area, "Recorded Range (Min / Max):", (col2_x, y_start), FONT_FACE, FONT_SCALE_INSTR,
                        (0, 255, 0), FONT_THICKNESS, cv2.LINE_AA)  # Green title
            y_offset = y_start + line_height + 5

            for i, internal_key in enumerate(INTERNAL_METRIC_KEYS):
                # Display Current Value
                current_val = current_metrics.get(internal_key)
                val_str = f"{current_val:.1f}" if current_val is not None else "N/A"
                cv2.putText(info_area, f"{internal_key}: {val_str}", (col1_x, y_offset + i * line_height), FONT_FACE,
                            FONT_SCALE_VALUE, (255, 255, 0), FONT_THICKNESS, cv2.LINE_AA)

                # Display Recorded Range
                min_val = calibration_ranges[internal_key]['min']
                max_val = calibration_ranges[internal_key]['max']
                min_val_str = f"{min_val:.1f}" if min_val != float('inf') else "N/A"
                max_val_str = f"{max_val:.1f}" if max_val != float('-inf') else "N/A"
                cv2.putText(info_area, f"{internal_key}: {min_val_str} / {max_val_str}",
                            (col2_x, y_offset + i * line_height), FONT_FACE, FONT_SCALE_VALUE, (0, 255, 0),
                            FONT_THICKNESS, cv2.LINE_AA)

            # Combine and display
            try:
                display_frame = np.vstack((info_area, color_image))
                cv2.imshow("Hand Angle Calibration (English)", display_frame)
            except Exception as display_e:
                print(f"Error displaying image: {display_e}")
                print(f"Info Area Shape: {info_area.shape}, Color Image Shape: {color_image.shape}")

            # 5. Handle Keys
            key = cv2.waitKey(5) & 0xFF
            if key == ord('q'):
                print("Quit key pressed.")
                break
            elif key == ord('r'):
                print("Resetting calibration ranges...")
                calibration_ranges = initialize_calibration_ranges()
                print("Ranges reset.")
            elif key == ord('s'):
                print("Saving current calibration ranges to JSON...")
                save_calibration_to_json(calibration_ranges, CALIBRATION_FILE)
                print_calibration_results(calibration_ranges)
            elif key == ord('o'):
                print("Info: Hold hand OPEN/STRAIGHT pose...")
            elif key == ord('c'):
                print("Info: Hold hand CLOSED/FIST pose...")
            elif key == ord('i'):
                print("Info: Hold thumb IN/ADDUCTED pose...")
            elif key == ord('t'):
                print("Info: Hold thumb OUT/ABDUCTED pose...")

    # --- Cleanup & Final Save ---
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Exiting...")
    except Exception as e:
        print(f"\nError during calibration: {e}")
        traceback.print_exc()
    finally:
        print("Cleaning up resources...")
        if orbbec_camera is not None:
            print("Stopping Orbbec pipeline...")
            try:
                orbbec_camera.stop()
            except:
                pass
        cv2.destroyAllWindows()
        print("OpenCV windows closed.")

        if 'calibration_ranges' in locals():
            print("Saving final calibration data before exiting...")
            save_calibration_to_json(calibration_ranges, CALIBRATION_FILE)
            print_calibration_results(calibration_ranges)
        print("Calibration program finished.")


# --- Program Entry Point ---
if __name__ == "__main__":
    run_calibration()
