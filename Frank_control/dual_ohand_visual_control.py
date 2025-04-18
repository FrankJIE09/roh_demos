# 文件名: dual_ohand_visual_control_v2.py (建议新文件名)
# 描述: 使用单个 Orbbec 相机同时检测左右手，并分别控制两个 OHand 灵巧手 (修正显示标签)。

import cv2
import mediapipe as mp
import numpy as np
import math
import time
import traceback

# --- 导入 Orbbec 相机模块 ---
try:
    from camera.orbbec_camera import OrbbecCamera, get_serial_numbers
    print("成功导入 'orbbec_camera' 模块。")
except ImportError as e:
    print(f"错误：无法导入 'orbbec_camera' 模块。Import Error: {e}")
    exit(1)
except Exception as e:
    print(f"导入 'orbbec_camera' 时发生未知错误: {e}")
    exit(1)

# --- 导入 OHand Modbus 客户端模块 ---
try:
    from ohand_modbus_client import (OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX,
                                     FINGER_MIDDLE, FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT)
    print("成功导入 'ohand_modbus_client' 模块。")
    print("提示：确保 'roh_registers_v1.py' 对 'ohand_modbus_client.py' 可用。")
except ImportError as e:
    print(f"错误：无法导入 'ohand_modbus_client' 或其依赖。Import Error: {e}")
    exit(1)
except Exception as e:
    print(f"导入 'ohand_modbus_client' 时发生未知错误: {e}")
    exit(1)


# --- 常量与配置 ---
# (保持之前的 OHand 配置, 映射范围, MediaPipe 初始化等不变)
# Orbbec 相机配置
ORBBEC_CAMERA_SN = None # 设置为 None 将使用找到的第一个设备，或指定序列号

# OHand Modbus 配置 (*** 请根据你的实际设置修改 ***)
OHAND_LEFT_PORT = '/dev/ttyUSB0'     # *** 左手 OHand 连接的串口 ***
OHAND_LEFT_ID = 2                   # *** 左手 OHand 的 Modbus Slave ID ***
OHAND_RIGHT_PORT = '/dev/ttyUSB1'    # *** 右手 OHand 连接的串口 ***
OHAND_RIGHT_ID = 2                  # *** 右手 OHand 的 Modbus Slave ID (必须与左手不同) ***
OHAND_BAUDRATE = 115200             # OHand 的 Modbus 波特率

# OHand 逻辑位置范围
OHAND_POS_OPEN = 0       # 手指完全张开时的逻辑位置值
OHAND_POS_CLOSED = 65535  # 手指完全闭合时的逻辑位置值
OHAND_THUMB_ROT_MIN = 65535     # 拇指旋转的一个极限逻辑位置值
OHAND_THUMB_ROT_MAX = 0 # 拇指旋转的另一个极限逻辑位置值

# --- 输入参数范围估计 (*** 需要通过观察和调试精确校准! ***) ---
BEND_DIST_MIN = 0.2  # 手指非常弯曲时的距离 (估计值)
BEND_DIST_MAX = 0.3  # 手指伸直时的距离 (估计值)
THUMB_ROT_DIST_MIN = 0.1# 拇指靠近手掌时的距离 (估计值)
THUMB_ROT_DIST_MAX = 0.15 # 拇指外展时的距离 (估计值)

# 其他常量
SEND_INTERVAL = 0.05
POS_CHANGE_THRESHOLD = 100
ESC_KEY = 27

# --- MediaPipe Hands 初始化 (检测双手) ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    model_complexity=0,
    max_num_hands=2,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# --- OHand Modbus 客户端 初始化 ---
ohand_left_client: OHandModbusClient | None = None
ohand_right_client: OHandModbusClient | None = None
try:
    print(f"初始化 左手 OHand Modbus (Port: {OHAND_LEFT_PORT}, ID: {OHAND_LEFT_ID})")
    ohand_left_client = OHandModbusClient(port=OHAND_LEFT_PORT, slave_id=OHAND_LEFT_ID, baudrate=OHAND_BAUDRATE)
except Exception as e:
    print(f"创建 左手 OHandModbusClient 失败: {e}")
try:
    print(f"初始化 右手 OHand Modbus (Port: {OHAND_RIGHT_PORT}, ID: {OHAND_RIGHT_ID})")
    ohand_right_client = OHandModbusClient(port=OHAND_RIGHT_PORT, slave_id=OHAND_RIGHT_ID, baudrate=OHAND_BAUDRATE)
except Exception as e:
    print(f"创建 右手 OHandModbusClient 失败: {e}")

# --- 工具函数 ---
def map_value(value, in_min, in_max, out_min, out_max):
    """将一个值从输入范围线性映射到输出范围"""
    if in_max == in_min: return out_min
    mapped = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if out_min <= out_max: return max(min(mapped, out_max), out_min)
    else: return max(min(mapped, out_min), out_max)

def calculate_distance(p1, p2):
    """计算两个 MediaPipe landmark 点之间的欧氏距离 (3D)"""
    if p1 is None or p2 is None: return float('inf')
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

# --- OHand 控制函数 ---
def send_to_ohand_individual(client: OHandModbusClient | None, target_positions: list):
    """使用指定的 OHand Modbus 客户端为 6 个自由度发送目标位置指令。"""
    if not (client and client.is_connected): return
    if len(target_positions) != 6:
        print(f"错误: target_positions 列表长度应为 6，实际为 {len(target_positions)}")
        return

    finger_indices = [
        FINGER_THUMB_BEND, FINGER_INDEX, FINGER_MIDDLE,
        FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT
    ]
    try:
        for i, finger_index in enumerate(finger_indices):
            target_pos_int = int(round(target_positions[i]))
            target_pos_int = max(0, min(target_pos_int, 65535)) # 限制范围
            client.set_finger_target_pos(finger_index, target_pos_int)
    except Exception as e:
        print(f"发送 OHand 命令时出错 (客户端: {client.client.port if client else 'N/A'}): {e}")

# --- 主执行逻辑 ---
def main():
    if ohand_left_client is None and ohand_right_client is None:
        print("错误：左右手 OHand Modbus 客户端均未能成功初始化，程序退出。")
        return

    orbbec_camera = None
    ohand_left_connected = False
    ohand_right_connected = False

    try:
        # --- 初始化 Orbbec 相机 ---
        print("查找奥比中光设备...")
        available_sns = get_serial_numbers()
        if not available_sns: print("错误：未找到奥比中光设备。"); return
        camera_sn = ORBBEC_CAMERA_SN if ORBBEC_CAMERA_SN else available_sns[0]
        print(f"尝试初始化 Orbbec 相机，序列号: {camera_sn if camera_sn else '第一个可用设备'}")
        orbbec_camera = OrbbecCamera(camera_sn if camera_sn else available_sns[0])
        print(f"Orbbec 相机 {orbbec_camera.get_serial_number()} 初始化成功。")

        # --- 连接 OHand 设备 ---
        if ohand_left_client:
            print(f"连接 左手 OHand ({OHAND_LEFT_PORT})...")
            if ohand_left_client.connect(): ohand_left_connected = True; print("左手 OHand 连接成功。")
            else: print(f"错误：连接左手 OHand ({OHAND_LEFT_PORT}) 失败。")
        if ohand_right_client:
            print(f"连接 右手 OHand ({OHAND_RIGHT_PORT})...")
            if ohand_right_client.connect(): ohand_right_connected = True; print("右手 OHand 连接成功。")
            else: print(f"错误：连接右手 OHand ({OHAND_RIGHT_PORT}) 失败。")

        if not (ohand_left_connected or ohand_right_connected):
            print("警告：没有任何 OHand 灵巧手连接成功，将仅进行视觉追踪。")

        # --- 启动 Orbbec 数据流 ---
        print("启动奥比中光相机数据流 (仅彩色)...")
        orbbec_camera.start_stream(color_stream=True, depth_stream=True, use_alignment=False, enable_sync=False)
        print("Orbbec 数据流已启动。")

        # --- 循环变量初始化 ---
        last_send_time = time.time()
        last_sent_positions_left = [OHAND_POS_OPEN] * 6
        last_sent_positions_right = [OHAND_POS_OPEN] * 6
        frame_counter = 0
        print("进入主循环 (在 OpecCV 窗口中按 'q' 退出)...")

        while True:
            frame_counter += 1
            # --- 1. 获取图像 ---
            color_image, _, _ = orbbec_camera.get_frames()
            if color_image is None: time.sleep(0.01); continue
            if not isinstance(color_image, np.ndarray) or color_image.dtype != np.uint8 or len(color_image.shape) != 3:
                 print(f"警告：图像格式不正确。"); continue

            # --- 2. MediaPipe 手部检测 (双手) ---
            image_height, image_width, _ = color_image.shape
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            results = hands.process(image_rgb)
            color_image.flags.writeable = True

            # 初始化当前帧的左右手目标位置
            current_target_positions_left = last_sent_positions_left[:]
            current_target_positions_right = last_sent_positions_right[:]
            detected_left_this_frame = False
            detected_right_this_frame = False

            # --- 3. 提取参数并映射 (区分左右手) ---
            if results.multi_hand_landmarks and results.multi_handedness:
                for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    handedness = results.multi_handedness[i].classification[0].label
                    score = results.multi_handedness[i].classification[0].score

                    mp_drawing.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                                              mp_drawing_styles.get_default_hand_landmarks_style(),
                                              mp_drawing_styles.get_default_hand_connections_style())

                    # --- *** 修改点：确定显示的标签和颜色 *** ---
                    display_label = ""
                    display_color = (255, 255, 255) # 默认白色
                    if handedness == 'Left':
                        # MediaPipe 识别为 'Left' -> 控制 右手 OHand
                        display_label = "right_hand" # 显示控制目标
                        display_color = (0, 255, 0) # 右手用绿色
                    elif handedness == 'Right':
                        # MediaPipe 识别为 'Right' -> 控制 左手 OHand
                        display_label = "left_hand" # 显示控制目标
                        display_color = (255, 0, 255) # 左手用洋红

                    # 在手上标注 (使用新的 display_label 和颜色)
                    cv2.putText(color_image, f"{display_label} ({score:.2f})",
                                (int(hand_landmarks.landmark[0].x * image_width) - 10, int(hand_landmarks.landmark[0].y * image_height) - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.8, display_color, 2) # 字体改大一点
                    # --- *** 修改结束 *** ---

                    landmarks = hand_landmarks.landmark
                    wrist = landmarks[mp_hands.HandLandmark.WRIST]
                    current_hand_targets = [OHAND_POS_OPEN] * 6

                    # --- 计算弯曲距离 ---
                    distances_bend = { finger_idx: calculate_distance(landmarks[tip_idx], wrist) for finger_idx, tip_idx in [
                        (FINGER_THUMB_BEND, mp_hands.HandLandmark.THUMB_TIP),
                        (FINGER_INDEX,      mp_hands.HandLandmark.INDEX_FINGER_TIP),
                        (FINGER_MIDDLE,     mp_hands.HandLandmark.MIDDLE_FINGER_TIP),
                        (FINGER_RING,       mp_hands.HandLandmark.RING_FINGER_TIP),
                        (FINGER_PINKY,      mp_hands.HandLandmark.PINKY_TIP)]}
                    # --- 计算拇指旋转距离 ---
                    thumb_rot_dist = calculate_distance(landmarks[mp_hands.HandLandmark.THUMB_TIP],
                                                        landmarks[mp_hands.HandLandmark.PINKY_MCP])

                    # --- 映射弯曲距离 ---
                    for finger_index, dist in distances_bend.items():
                        if dist != float('inf'):
                            mapped_pos = map_value(dist, BEND_DIST_MIN, BEND_DIST_MAX, OHAND_POS_CLOSED, OHAND_POS_OPEN)
                            current_hand_targets[finger_index] = mapped_pos

                    # --- 映射拇指旋转距离 ---
                    if thumb_rot_dist != float('inf'):
                        mapped_rot_pos = map_value(thumb_rot_dist, THUMB_ROT_DIST_MIN, THUMB_ROT_DIST_MAX, OHAND_THUMB_ROT_MIN, OHAND_THUMB_ROT_MAX)
                        current_hand_targets[FINGER_THUMB_ROT] = mapped_rot_pos

                    # --- 更新对应手的目标列表 (交换逻辑保持不变) ---
                    if handedness == 'Left':
                        current_target_positions_right = current_hand_targets[:]
                        detected_right_this_frame = True
                    elif handedness == 'Right':
                        current_target_positions_left = current_hand_targets[:]
                        detected_left_this_frame = True

            # --- 4. 发送指令到 OHand (区分左右手 - 发送逻辑保持不变) ---
            current_time = time.time()
            time_elapsed = current_time - last_send_time
            should_send = time_elapsed > SEND_INTERVAL
            left_pos_changed = False
            if detected_left_this_frame:
                for i in range(6):
                    if abs(round(current_target_positions_left[i]) - round(last_sent_positions_left[i])) > POS_CHANGE_THRESHOLD:
                        left_pos_changed = True; break
            right_pos_changed = False
            if detected_right_this_frame:
                 for i in range(6):
                    if abs(round(current_target_positions_right[i]) - round(last_sent_positions_right[i])) > POS_CHANGE_THRESHOLD:
                        right_pos_changed = True; break

            if should_send or left_pos_changed or right_pos_changed:
                if ohand_left_connected and detected_left_this_frame:
                    send_to_ohand_individual(ohand_left_client, current_target_positions_left)
                    last_sent_positions_left = current_target_positions_left[:]
                if ohand_right_connected and detected_right_this_frame:
                    send_to_ohand_individual(ohand_right_client, current_target_positions_right)
                    last_sent_positions_right = current_target_positions_right[:]

                last_send_time = current_time

            # --- 5. 显示图像 ---
            cv2.imshow('Orbbec OHand', color_image)

            # --- 6. 处理按键 ---
            key = cv2.waitKey(5) & 0xFF
            if key == ord('q') or key == ESC_KEY:
                print("退出键被按下。")
                break

    # --- 异常处理与清理 ---
    except KeyboardInterrupt: print("\n检测到 Ctrl+C。正在退出...")
    except Exception as e:
        print(f"\n主循环发生未处理的错误: {e}")
        traceback.print_exc()
    finally:
        print("正在清理资源...")
        # (清理部分保持不变)
        if orbbec_camera is not None:
            print("停止 Orbbec pipeline...")
            try: orbbec_camera.stop()
            except Exception as e: print(f"停止 Orbbec camera 时出错: {e}")
        if ohand_left_client is not None and ohand_left_client.is_connected:
            print("断开 左手 OHand 连接...")
            try: ohand_left_client.disconnect()
            except Exception as e: print(f"断开 左手 OHand 时出错: {e}")
        if ohand_right_client is not None and ohand_right_client.is_connected:
            print("断开 右手 OHand 连接...")
            try: ohand_right_client.disconnect()
            except Exception as e: print(f"断开 右手 OHand 时出错: {e}")
        cv2.destroyAllWindows()
        print("OpenCV 窗口已关闭。")
        print("程序结束。")

# --- 程序入口 ---
if __name__ == "__main__":
    main()