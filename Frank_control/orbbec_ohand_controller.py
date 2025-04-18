# 文件名: orbbec_ohand_controller_individual.py
# 描述: 使用 Orbbec 相机进行手部追踪，并将各手指参数映射到 OHand 对应自由度进行控制。

import cv2
import mediapipe as mp
import numpy as np
import math
import time
import traceback
from camera.orbbec_camera import OrbbecCamera, get_serial_numbers

# --- 导入 Orbbec Camera 模块 ---
try:
    from camera.orbbec_camera import OrbbecCamera, get_serial_numbers
    print("成功导入 'orbbec_camera' 模块。")
except ImportError as e:
    print(f"错误：无法导入 'orbbec_camera' 模块。Import Error: {e}")
    exit(1) # 缺少核心模块则退出

# --- 导入 OHand Modbus 客户端模块 ---
try:
    from ohand_modbus_client import (OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX,
                                     FINGER_MIDDLE, FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT)
    print("成功导入 'ohand_modbus_client' 模块。")
    print("提示：确保 'roh_registers_v1.py' 对 'ohand_modbus_client.py' 可用。")
except ImportError as e:
    print(f"错误：无法导入 'ohand_modbus_client' 或其依赖。Import Error: {e}")
    exit(1) # 缺少核心模块则退出


# --- 常量与配置 ---
# OHand Modbus 配置 (*** 请根据你的实际设置修改 ***)
OHAND_SERIAL_PORT = '/dev/ttyUSB0'  # OHand 串口
OHAND_SLAVE_ID = 2                 # OHand Modbus ID
OHAND_BAUDRATE = 115200            # OHand 波特率

# OHand 逻辑位置范围 (0-65535)
OHAND_POS_OPEN = 0       # OHand 手指完全张开时的逻辑位置值
OHAND_POS_CLOSED = 65535  # OHand 手指完全闭合时的逻辑位置值
# OHand 拇指旋转范围 (需要根据实际情况校准)
OHAND_THUMB_ROT_MIN = 65535     # 拇指旋转到一个方向的逻辑位置值 (例如内收)
OHAND_THUMB_ROT_MAX = 0 # 拇指旋转到另一个方向的逻辑位置值 (例如外展)

# --- 输入参数范围估计 (*** 这些值需要通过观察和调试来精确校准! ***) ---
# 指尖到手腕距离 (用于弯曲): 数值越小越弯曲
# 假设归一化坐标系下的大致范围 (需要观察实际值)
BEND_DIST_MIN = 0.2  # 手指非常弯曲时的距离 (估计值)
BEND_DIST_MAX = 0.3  # 手指伸直时的距离 (估计值)

# 拇指指尖到小指根部距离 (用于旋转): 数值越大越外展
# 假设归一化坐标系下的大致范围 (需要观察实际值)
THUMB_ROT_DIST_MIN = 0.1 # 拇指靠近手掌时的距离 (估计值)
THUMB_ROT_DIST_MAX = 0.2 # 拇指外展时的距离 (估计值)

# 其他常量
SEND_INTERVAL = 0.05 # 发送指令最小间隔 (秒)
ESC_KEY = 27         # OpenCV 退出键

# --- MediaPipe Hands 初始化 ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    model_complexity=0,
    max_num_hands=1,
    min_detection_confidence=0.7, # 提高置信度以获得更稳定的点
    min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# --- OHand Modbus Client 初始化 ---
ohand_client: OHandModbusClient | None = None
try:
    print(f"初始化 OHand Modbus (Port: {OHAND_SERIAL_PORT}, ID: {OHAND_SLAVE_ID}, Baud: {OHAND_BAUDRATE})")
    ohand_client = OHandModbusClient(port=OHAND_SERIAL_PORT, slave_id=OHAND_SLAVE_ID, baudrate=OHAND_BAUDRATE)
except Exception as e:
    print(f"创建 OHandModbusClient 实例失败: {e}")
    ohand_client = None

# --- 工具函数 ---
def map_value(value, in_min, in_max, out_min, out_max):
    """将一个值从输入范围线性映射到输出范围"""
    if in_max == in_min: return out_min
    mapped = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    if out_min <= out_max: return max(min(mapped, out_max), out_min)
    else: return max(min(mapped, out_min), out_max)

def calculate_distance(p1, p2):
    """计算两个 MediaPipe landmark 点之间的欧氏距离 (使用 x, y, z 归一化坐标)"""
    if p1 is None or p2 is None: return float('inf') # 返回无穷大表示无效
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

# --- OHand 控制函数 (修改后) ---
def send_to_ohand_individual(target_positions: list):
    """
    使用 OHand Modbus 客户端为 6 个自由度分别发送目标位置指令。

    :param target_positions: 包含 6 个目标逻辑位置 (0-65535) 的列表，顺序必须是：
                             [ThumbBend, Index, Middle, Ring, Pinky, ThumbRot]
    """
    if not (ohand_client and ohand_client.is_connected):
        # print("OHand 未连接或未初始化") # 过于频繁
        return

    if len(target_positions) != 6:
        print(f"错误: target_positions 列表长度应为 6，实际为 {len(target_positions)}")
        return

    finger_indices = [
        FINGER_THUMB_BEND, FINGER_INDEX, FINGER_MIDDLE,
        FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT
    ]

    try:
        success_count = 0
        for i, finger_index in enumerate(finger_indices):
            target_pos_int = int(round(target_positions[i]))
            # 检查目标值是否在合理范围内
            if not (0 <= target_pos_int <= 65535):
                 # print(f"警告: 手指 {finger_index} 的目标位置 {target_pos_int} 超出范围 [0, 65535]，已限制。")
                 target_pos_int = max(0, min(target_pos_int, 65535)) # 限制在范围内

            # print(f"--- 发送 OHand 手指 {finger_index} 到位置: {target_pos_int} ---") # 调试
            if ohand_client.set_finger_target_pos(finger_index, target_pos_int):
                success_count += 1
            # else:
            #     print(f"  设置手指 {finger_index} 失败。") # 调试

        # if success_count != 6:
        #      print(f"警告：并非所有手指指令都发送成功 ({success_count}/6)")

    except Exception as e:
        print(f"发送 OHand 命令时出错: {e}")


# --- 主执行逻辑 ---
def main():
    # 检查 OHand 客户端是否初始化成功
    if ohand_client is None:
        print("错误：OHand Modbus 客户端未能成功初始化，程序退出。")
        return

    orbbec_camera = None      # Orbbec 相机实例
    ohand_connected = False   # OHand 连接状态标志

    try:
        # --- 初始化 Orbbec 相机 ---
        print("查找奥比中光设备...")
        available_sns = get_serial_numbers()
        if not available_sns: print("错误：未找到奥比中光设备。"); return
        camera_sn = available_sns[0]
        print(f"尝试初始化 Orbbec 相机，序列号: {camera_sn}")
        orbbec_camera = OrbbecCamera(camera_sn)
        print(f"Orbbec 相机 {camera_sn} 初始化成功。")

        # --- 连接 OHand 设备 ---
        print("连接到 OHand 设备...")
        if ohand_client.connect():
            ohand_connected = True
            print("OHand 连接成功。")
            # 可选: 设置初始速度等
            # default_speed = 30000
            # for i in range(6): ohand_client.set_finger_speed(i, default_speed)
        else:
            print("错误：连接 OHand 设备失败。")
            # return # 可根据需要决定是否退出

        # --- 启动 Orbbec 数据流 ---
        print("启动奥比中光相机数据流 (仅彩色)...")
        orbbec_camera.start_stream(color_stream=True, depth_stream=True, use_alignment=False, enable_sync=False)
        print("Orbbec 数据流已启动。")

        # --- 循环变量初始化 ---
        last_send_time = time.time()
        # 初始化上次发送的目标位置列表 (例如，全部设为张开状态)
        last_sent_positions = [OHAND_POS_OPEN] * 6
        frame_counter = 0
        print("进入主循环 (在 OpecCV 窗口中按 'q' 退出)...")

        while True:
            frame_counter += 1
            # --- 1. 获取图像 ---
            color_image, _, _ = orbbec_camera.get_frames()
            if color_image is None: time.sleep(0.01); continue
            if not isinstance(color_image, np.ndarray) or color_image.dtype != np.uint8 or len(color_image.shape) != 3:
                print(f"警告：图像格式不正确。"); continue

            # --- 2. MediaPipe 手部检测 ---
            image_height, image_width, _ = color_image.shape
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False
            results = hands.process(image_rgb)
            color_image.flags.writeable = True

            # 初始化当前帧的目标位置列表 (默认设为张开)
            current_target_positions = [OHAND_POS_OPEN] * 6

            # --- 3. 提取参数并映射 ---
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks: # 只处理第一只手
                    mp_drawing.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                                              mp_drawing_styles.get_default_hand_landmarks_style(),
                                              mp_drawing_styles.get_default_hand_connections_style())

                    landmarks = hand_landmarks.landmark
                    wrist = landmarks[mp_hands.HandLandmark.WRIST]

                    # --- 计算各手指弯曲距离 (指尖到手腕) ---
                    distances_bend = {
                        FINGER_THUMB_BEND: calculate_distance(landmarks[mp_hands.HandLandmark.THUMB_TIP], wrist),
                        FINGER_INDEX:      calculate_distance(landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP], wrist),
                        FINGER_MIDDLE:     calculate_distance(landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP], wrist),
                        FINGER_RING:       calculate_distance(landmarks[mp_hands.HandLandmark.RING_FINGER_TIP], wrist),
                        FINGER_PINKY:      calculate_distance(landmarks[mp_hands.HandLandmark.PINKY_TIP], wrist),
                    }

                    # --- 计算拇指旋转距离 (拇指尖到小指根部) ---
                    thumb_rot_dist = calculate_distance(landmarks[mp_hands.HandLandmark.THUMB_TIP],
                                                        landmarks[mp_hands.HandLandmark.PINKY_MCP])

                    # --- 映射弯曲距离到 OHand 位置 ---
                    # 距离越小 -> 越弯曲 -> OHand 位置越接近 CLOSED
                    for finger_index, dist in distances_bend.items():
                        if dist != float('inf'): # 检查距离是否有效
                            # 输入范围 BEND_DIST_MIN 到 BEND_DIST_MAX (小->大)
                            # 输出范围 OHAND_POS_CLOSED 到 OHAND_POS_OPEN (大->小)
                            mapped_pos = map_value(dist, BEND_DIST_MIN, BEND_DIST_MAX,
                                                   OHAND_POS_CLOSED, OHAND_POS_OPEN)
                            current_target_positions[finger_index] = mapped_pos
                            # 可选：在屏幕上显示每个手指的距离和映射后的位置值
                            # cv2.putText(color_image, f"F{finger_index} D:{dist:.2f} P:{int(mapped_pos)}", (10, 120 + finger_index * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)


                    # --- 映射拇指旋转距离到 OHand 位置 ---
                    # 距离越大 -> 越外展 -> OHand 位置越接近 ROT_MAX
                    if thumb_rot_dist != float('inf'):
                        # 输入范围 THUMB_ROT_DIST_MIN 到 THUMB_ROT_DIST_MAX (小->大)
                        # 输出范围 OHAND_THUMB_ROT_MIN 到 OHAND_THUMB_ROT_MAX (小->大 或 大->小，取决于校准)
                        mapped_rot_pos = map_value(thumb_rot_dist, THUMB_ROT_DIST_MIN, THUMB_ROT_DIST_MAX,
                                                   OHAND_THUMB_ROT_MIN, OHAND_THUMB_ROT_MAX)
                        current_target_positions[FINGER_THUMB_ROT] = mapped_rot_pos
                        # 可选：显示拇指旋转距离和映射位置
                        # cv2.putText(color_image, f"TR D:{thumb_rot_dist:.2f} P:{int(mapped_rot_pos)}", (10, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

                    break # 只处理第一只检测到的手

            # --- 4. 发送指令到 OHand ---
            current_time = time.time()
            # 检查是否有任何一个手指的目标位置发生了显著变化，或者是否到达发送间隔
            position_changed = False
            for i in range(6):
                 # 使用整数比较避免浮点误差，阈值设为 100 (可调整)
                 if abs(round(current_target_positions[i]) - round(last_sent_positions[i])) > 100:
                      position_changed = True
                      break

            if ohand_connected and (position_changed or (current_time - last_send_time) > SEND_INTERVAL):
                # 调用新的控制函数，传入包含 6 个目标位置的列表
                send_to_ohand_individual(current_target_positions)
                # 更新上次发送的位置记录
                last_sent_positions = current_target_positions[:] # 使用切片创建副本
                last_send_time = current_time

            # --- 5. 显示图像 ---
            cv2.imshow('Orbbec + OHand', color_image)

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
        if orbbec_camera is not None:
            print("停止 Orbbec pipeline...")
            try: orbbec_camera.stop()
            except Exception as e: print(f"停止 Orbbec camera 时出错: {e}")
        if ohand_client is not None and ohand_client.is_connected:
            print("断开 OHand 连接...")
            try:
                 # 可选: 发送安全位置指令
                 # safe_positions = [OHAND_POS_OPEN] * 6 # 例如全部张开
                 # send_to_ohand_individual(safe_positions)
                 # time.sleep(1.0)
                 ohand_client.disconnect()
                 print("OHand 连接已断开。")
            except Exception as e: print(f"断开 OHand 时出错: {e}")
        cv2.destroyAllWindows()
        print("OpenCV 窗口已关闭。")
        print("程序结束。")

# --- 程序入口 ---
if __name__ == "__main__":
    main()