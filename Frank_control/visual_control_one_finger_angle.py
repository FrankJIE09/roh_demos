#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 导入必要的库
import cv2
import mediapipe as mp
import numpy as np
import time
import yaml

from camera.orbbec_camera import OrbbecCamera, get_serial_numbers, close_connected_cameras

try:
    from ohand_modbus_client import OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX, FINGER_MIDDLE, FINGER_RING, \
        FINGER_PINKY, FINGER_THUMB_ROT
except ImportError:
    print("错误：未找到 ohand_modbus_client.py 或其依赖项 (如 roh_registers_v1.py)。")
    print("请确保这些文件在正确的目录中。")
    exit(1)

# --- 配置常量 (将从 config.yaml 加载) ---
OHAND_SERIAL_PORT = ''
OHAND_SLAVE_ID = 0

# 更新和新增的角度映射参数
VISUAL_THUMB_BEND_INPUT_MIN = 0
VISUAL_THUMB_BEND_INPUT_MAX = 0
VISUAL_OTHER_FINGERS_BEND_INPUT_MIN = 0
VISUAL_OTHER_FINGERS_BEND_INPUT_MAX = 0
VISUAL_THUMB_ROT_INPUT_MIN = 0
VISUAL_THUMB_ROT_INPUT_MAX = 0

OHAND_THUMB_BEND_RANGE = (0, 0)
OHAND_FINGER_BEND_RANGE = (0, 0)
OHAND_THUMB_ROT_RANGE = (0, 0)

MP_MIN_DETECTION_CONFIDENCE = 0.0
MP_MIN_TRACKING_CONFIDENCE = 0.0
MP_MAX_NUM_HANDS = 0
OHAND_SEND_INTERVAL = 0.0
ORBBEC_CAMERA_SERIAL_NUMBER = ''


def load_config(config_path='config.yaml'):
    """加载配置文件."""
    global OHAND_SERIAL_PORT, OHAND_SLAVE_ID, \
        VISUAL_THUMB_BEND_INPUT_MIN, VISUAL_THUMB_BEND_INPUT_MAX, \
        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX, \
        VISUAL_THUMB_ROT_INPUT_MIN, VISUAL_THUMB_ROT_INPUT_MAX, \
        OHAND_THUMB_BEND_RANGE, OHAND_FINGER_BEND_RANGE, OHAND_THUMB_ROT_RANGE, \
        MP_MIN_DETECTION_CONFIDENCE, MP_MIN_TRACKING_CONFIDENCE, \
        MP_MAX_NUM_HANDS, OHAND_SEND_INTERVAL, ORBBEC_CAMERA_SERIAL_NUMBER

    try:
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)

        # OHand 配置
        ohand_config = config.get('ohand_settings', {})
        OHAND_SERIAL_PORT = ohand_config.get('serial_port', '/dev/ttyUSB0')
        OHAND_SLAVE_ID = ohand_config.get('slave_id', 2)
        OHAND_SEND_INTERVAL = ohand_config.get('send_interval_seconds', 0.1)

        # 角度映射配置
        angle_map_config = config.get('angle_mapping', {})
        # 拇指弯曲视觉输入范围
        VISUAL_THUMB_BEND_INPUT_MIN = angle_map_config.get('visual_thumb_bend_input_min', 0)
        VISUAL_THUMB_BEND_INPUT_MAX = angle_map_config.get('visual_thumb_bend_input_max', 65)
        # 其他手指弯曲视觉输入范围 (原 visual_angle_input_min/max)
        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN = angle_map_config.get('visual_other_fingers_bend_input_min', 0)
        VISUAL_OTHER_FINGERS_BEND_INPUT_MAX = angle_map_config.get('visual_other_fingers_bend_input_max', 65)
        # 拇指旋转视觉输入范围
        VISUAL_THUMB_ROT_INPUT_MIN = angle_map_config.get('visual_thumb_rot_input_min', -90)
        VISUAL_THUMB_ROT_INPUT_MAX = angle_map_config.get('visual_thumb_rot_input_max', 90)

        OHAND_THUMB_BEND_RANGE = tuple(angle_map_config.get('ohand_thumb_bend_range', [0, 90]))
        OHAND_FINGER_BEND_RANGE = tuple(angle_map_config.get('ohand_finger_bend_range', [0, 135]))
        OHAND_THUMB_ROT_RANGE = tuple(angle_map_config.get('ohand_thumb_rot_range', [-45, 45]))

        # MediaPipe Hands 配置
        mediapipe_config = config.get('mediapipe_hands_settings', {})
        MP_MIN_DETECTION_CONFIDENCE = mediapipe_config.get('min_detection_confidence', 0.8)
        MP_MIN_TRACKING_CONFIDENCE = mediapipe_config.get('min_tracking_confidence', 0.5)
        MP_MAX_NUM_HANDS = mediapipe_config.get('max_num_hands', 1)

        # Orbbec Camera 配置
        orbbec_config = config.get('orbbec_camera_settings', {})
        ORBBEC_CAMERA_SERIAL_NUMBER = orbbec_config.get('serial_number', '')
        if not ORBBEC_CAMERA_SERIAL_NUMBER:
            print("警告：config.yaml 中未指定 Orbbec 摄像头序列号。将尝试自动检测第一个可用的摄像头。")
            available_sns = get_serial_numbers()
            if available_sns:
                ORBBEC_CAMERA_SERIAL_NUMBER = available_sns[0]
                print(f"已检测到并使用序列号为：{ORBBEC_CAMERA_SERIAL_NUMBER} 的摄像头。")
            else:
                print("错误：未找到任何 Orbbec 摄像头。请连接摄像头或在 config.yaml 中指定正确的序列号。")
                exit(1)

        print("配置加载成功。")
    except FileNotFoundError:
        print(f"错误：配置文件 {config_path} 未找到。请确保该文件存在。")
        exit(1)
    except yaml.YAMLError as e:
        print(f"错误：解析配置文件 {config_path} 时出错：{e}")
        exit(1)
    except Exception as e:
        print(f"加载配置时发生意外错误：{e}")
        exit(1)


# --- 辅助函数 ---

def is_palm_facing_camera(hand_landmarks):
    """
    判断掌心是否朝向摄像头。
    以 WRIST(0), INDEX_MCP(5), PINKY_MCP(17) 三点构成掌心平面，
    计算法向量Z分量，Z<0认为掌心朝向摄像头。
    """
    wrist = np.array([
        hand_landmarks.landmark[0].x,
        hand_landmarks.landmark[0].y,
        hand_landmarks.landmark[0].z
    ])
    index_mcp = np.array([
        hand_landmarks.landmark[5].x,
        hand_landmarks.landmark[5].y,
        hand_landmarks.landmark[5].z
    ])
    pinky_mcp = np.array([
        hand_landmarks.landmark[17].x,
        hand_landmarks.landmark[17].y,
        hand_landmarks.landmark[17].z
    ])
    v1 = index_mcp - wrist
    v2 = pinky_mcp - wrist
    palm_normal = np.cross(v1, v2)
    return palm_normal[2] < 0  # True: 掌心朝向摄像头


def translate_angle(value, visual_min, visual_max, ohand_min, ohand_max):
    """
    将一个值从视觉角度范围线性映射到 OHand 角度范围。
    :param value: 要映射的视觉角度值。
    :param visual_min: 视觉角度的最小值。
    :param visual_max: 视觉角度的最大值。
    :param ohand_min: OHand 角度的最小值。
    :param ohand_max: OHand 角度的最大值。
    :return: 映射后的 OHand 角度值。
    """
    if visual_max == visual_min:  # 避免除以零
        return ohand_min if value <= visual_min else ohand_max  # 或者返回中间值 (ohand_min + ohand_max) / 2

    visual_span = visual_max - visual_min
    ohand_span = ohand_max - ohand_min
    value_scaled = float(value - visual_min) / float(visual_span)
    translated_value = ohand_min + (value_scaled * ohand_span)
    return max(min(translated_value, ohand_max), ohand_min)


def compute_thumb_rotation_angle(hand_landmarks, image_width, image_height):
    """
    计算拇指的内外旋角度。
    """
    WRIST = 0
    THUMB_CMC = 1
    THUMB_MCP = 2
    THUMB_TIP = 4
    INDEX_MCP = 5

    if not hand_landmarks:
        return None

    # 简化：使用 Thumb_MCP (2), Thumb_Tip (4) 向量与 Wrist (0), Index_MCP (5) 向量的2D夹角
    vec_mcp_tip_2d = np.array([hand_landmarks.landmark[THUMB_TIP].x, hand_landmarks.landmark[THUMB_TIP].y]) - \
                     np.array([hand_landmarks.landmark[THUMB_MCP].x, hand_landmarks.landmark[THUMB_MCP].y])

    vec_wrist_index_mcp_2d = np.array([hand_landmarks.landmark[INDEX_MCP].x, hand_landmarks.landmark[INDEX_MCP].y]) - \
                             np.array([hand_landmarks.landmark[WRIST].x, hand_landmarks.landmark[WRIST].y])

    if np.linalg.norm(vec_mcp_tip_2d) == 0 or np.linalg.norm(vec_wrist_index_mcp_2d) == 0:
        return 0

    dot_product = np.dot(vec_mcp_tip_2d, vec_wrist_index_mcp_2d)
    norm_product = np.linalg.norm(vec_mcp_tip_2d) * np.linalg.norm(vec_wrist_index_mcp_2d)

    # 确保 dot_product / norm_product 在 [-1, 1] 范围内以避免 acos 错误
    value_for_acos = np.clip(dot_product / norm_product, -1.0, 1.0)
    angle_rad = np.arccos(value_for_acos)
    angle_deg = np.degrees(angle_rad)

    cross_product = vec_mcp_tip_2d[0] * vec_wrist_index_mcp_2d[1] - vec_mcp_tip_2d[1] * vec_wrist_index_mcp_2d[0]
    if cross_product < 0:
        angle_deg = -angle_deg

    # 注意：这里返回的是原始计算角度，映射将在主循环中根据配置的 VISUAL_THUMB_ROT_INPUT_MIN/MAX 进行
    return int(angle_deg)


def compute_finger_angles_from_visual(image, results, joint_list):
    """
    根据 MediaPipe 检测到的手部关键点计算手指的“视觉角度”和拇指旋转角度。
    :param image: 当前视频帧。
    :param results: MediaPipe hands.process() 的输出结果。
    :param joint_list: 定义了计算每个手指弯曲角度所需的三个关节点索引。
    :return: (带有调试信息的图像, 计算出的5个视觉弯曲角度列表, 1个视觉旋转角度)
    """
    visual_bend_angles = []
    visual_thumb_rot_angle = 0  # 初始化

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:  # 通常只处理第一只手
            # 计算5个手指的弯曲角度
            for i, joint_indices in enumerate(joint_list):
                a_coords = np.array(
                    [hand_landmarks.landmark[joint_indices[0]].x, hand_landmarks.landmark[joint_indices[0]].y])
                b_coords = np.array(
                    [hand_landmarks.landmark[joint_indices[1]].x, hand_landmarks.landmark[joint_indices[1]].y])
                c_coords = np.array(
                    [hand_landmarks.landmark[joint_indices[2]].x, hand_landmarks.landmark[joint_indices[2]].y])

                radians = np.arctan2(c_coords[1] - b_coords[1], c_coords[0] - b_coords[0]) - \
                          np.arctan2(a_coords[1] - b_coords[1], a_coords[0] - b_coords[0])
                angle_deg = np.abs(radians * 180.0 / np.pi)

                if angle_deg > 180.0:
                    angle_deg = 360.0 - angle_deg

                processed_angle = 0
                if i == 0:  # 拇指弯曲 (Finger index 0)
                    clamped_angle_deg = max(90, min(angle_deg, 180))  # 原始计算角度范围
                    # 插值到自定义的视觉输出范围，例如 [0, 200] 是拇指的一个较大的动态范围
                    processed_angle = np.interp(clamped_angle_deg, [90, 180], [0, 200])
                    # 根据配置文件中的拇指特定范围进行钳位
                    processed_angle = min(VISUAL_THUMB_BEND_INPUT_MAX, processed_angle)
                    processed_angle = max(VISUAL_THUMB_BEND_INPUT_MIN, processed_angle)
                else:  # 其他四指弯曲
                    clamped_angle_deg = max(30, min(angle_deg, 180))  # 原始计算角度范围
                    # 插值到自定义的视觉输出范围，例如 [0, 180]
                    processed_angle = np.interp(clamped_angle_deg, [30, 180], [0, 180])
                    # 根据配置文件中的其他手指特定范围进行钳位
                    processed_angle = min(VISUAL_OTHER_FINGERS_BEND_INPUT_MAX, processed_angle)
                    processed_angle = max(VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, processed_angle)

                visual_bend_angles.append(int(processed_angle))

                debug_text_pos = tuple(np.multiply(b_coords, [image.shape[1], image.shape[0]]).astype(int))
                cv2.putText(image, f"{i}:{int(angle_deg):.0f} > {int(processed_angle)}",
                            debug_text_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (50, 50, 200), 2, cv2.LINE_AA)

            # 计算拇指旋转角度
            temp_rot_angle = compute_thumb_rotation_angle(hand_landmarks, image.shape[1], image.shape[0])
            if temp_rot_angle is not None:
                visual_thumb_rot_angle = temp_rot_angle  # 这个值将直接用于 translate_angle，由配置的输入范围处理

                # 显示拇指旋转调试信息 (原始计算角度)
                thumb_mcp_coords = np.array(
                    [hand_landmarks.landmark[2].x, hand_landmarks.landmark[2].y])
                debug_rot_pos = tuple(
                    np.multiply(thumb_mcp_coords, [image.shape[1], image.shape[0]]).astype(int) + np.array(
                        [0, 20]))
                cv2.putText(image, f"ThumbRotRaw: {visual_thumb_rot_angle} deg",
                            debug_rot_pos,
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 50, 50), 2, cv2.LINE_AA)

            break  # 只处理检测到的第一只手

    return image, visual_bend_angles, visual_thumb_rot_angle


# --- 主执行函数 ---
def main():
    load_config()

    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands(
        max_num_hands=MP_MAX_NUM_HANDS,
        min_detection_confidence=MP_MIN_DETECTION_CONFIDENCE,
        min_tracking_confidence=MP_MIN_TRACKING_CONFIDENCE)
    mp_drawing = mp.solutions.drawing_utils

    joint_list = [
        [4, 3, 2],  # 拇指弯曲
        [8, 6, 5],  # 食指弯曲
        [12, 10, 9],  # 中指弯曲
        [16, 14, 13],  # 无名指弯曲
        [20, 18, 17]  # 小指弯曲
    ]

    print(f"正在初始化 OHand 客户端，端口 {OHAND_SERIAL_PORT}...")
    ohand = OHandModbusClient(port=OHAND_SERIAL_PORT, slave_id=OHAND_SLAVE_ID)
    if not ohand.connect():
        print(f"连接 OHand 失败 (端口: {OHAND_SERIAL_PORT})。正在退出。")
        return
    print("OHand 连接成功。")

    print(f"正在初始化 Orbbec 摄像头，序列号 {ORBBEC_CAMERA_SERIAL_NUMBER}...")
    camera = None
    try:
        camera = OrbbecCamera(ORBBEC_CAMERA_SERIAL_NUMBER)
        camera.start_stream(depth_stream=True, color_stream=True, enable_sync=False, use_alignment=False)
        print("Orbbec 摄像头流启动成功。")
    except Exception as e:
        print(f"错误：无法初始化或启动 Orbbec 摄像头：{e}")
        if ohand.is_connected:
            ohand.disconnect()
        return

    print("手部追踪已启动。按 'q' 键退出。")
    last_sent_time = time.time()

    # 设置窗口为全屏，只需设置一次
    cv2.namedWindow('OHand Visual Control', cv2.WINDOW_NORMAL)
    cv2.setWindowProperty('OHand Visual Control', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while True:
            color_image, _, _ = camera.get_frames()
            if color_image is None:
                print("错误：无法从 Orbbec 摄像头获取帧。")  # 稍微修改了错误信息
                time.sleep(0.1)  # 避免快速循环错误
                continue

            image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image = cv2.flip(image, 1)

            image.flags.writeable = False
            results = hands.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            ohand_target_angles = {}

            # === 新增：左右手判断与拇指旋转方向修正 ===
            if results.multi_hand_landmarks and results.multi_handedness:
                # 只处理第一只手
                hand_landmarks = results.multi_hand_landmarks[0]
                handedness = results.multi_handedness[0]
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                    mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2, circle_radius=3),
                    mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
                )
                # 计算角度
                image, visual_bend_angles, visual_thumb_rot_angle = compute_finger_angles_from_visual(image, results, joint_list)
                # handedness.classification[0].label: 'Left' 或 'Right'
                if handedness.classification[0].label == 'Left':
                    visual_thumb_rot_angle = visual_thumb_rot_angle  # 左手先取反
                if is_palm_facing_camera(hand_landmarks):
                    visual_thumb_rot_angle = -visual_thumb_rot_angle  # 掌心朝向摄像头再取反
                # 后续映射与显示
                if len(visual_bend_angles) == 5:
                    ohand_target_angles[FINGER_THUMB_BEND] = translate_angle(
                        visual_bend_angles[0],
                        VISUAL_THUMB_BEND_INPUT_MIN, VISUAL_THUMB_BEND_INPUT_MAX,
                        OHAND_THUMB_BEND_RANGE[0], OHAND_THUMB_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_INDEX] = translate_angle(
                        visual_bend_angles[1],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_MIDDLE] = translate_angle(
                        visual_bend_angles[2],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_RING] = translate_angle(
                        visual_bend_angles[3],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_PINKY] = translate_angle(
                        visual_bend_angles[4],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                if visual_thumb_rot_angle is not None:
                    ohand_target_angles[FINGER_THUMB_ROT] = translate_angle(
                        visual_thumb_rot_angle,
                        VISUAL_THUMB_ROT_INPUT_MIN, VISUAL_THUMB_ROT_INPUT_MAX,
                        OHAND_THUMB_ROT_RANGE[0], OHAND_THUMB_ROT_RANGE[1]
                    )
                    cv2.putText(image, f"OHandThumbRotTgt: {ohand_target_angles.get(FINGER_THUMB_ROT, 0.0):.1f}",
                                (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 1, cv2.LINE_AA)
                y_offset = 20
                if FINGER_THUMB_BEND in ohand_target_angles:
                    cv2.putText(image, f"OHandThBend: {ohand_target_angles[FINGER_THUMB_BEND]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_INDEX in ohand_target_angles:
                    cv2.putText(image, f"OHandIdxBend: {ohand_target_angles[FINGER_INDEX]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_MIDDLE in ohand_target_angles:
                    cv2.putText(image, f"OHandMidBend: {ohand_target_angles[FINGER_MIDDLE]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_RING in ohand_target_angles:
                    cv2.putText(image, f"OHandRngBend: {ohand_target_angles[FINGER_RING]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_PINKY in ohand_target_angles:
                    cv2.putText(image, f"OHandPnkBend: {ohand_target_angles[FINGER_PINKY]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
            # === 兼容旧逻辑：如果没有 handedness 信息，保持原有处理方式 ===
            elif results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                        mp_drawing.DrawingSpec(color=(255, 0, 0), thickness=2, circle_radius=3),
                        mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=2)
                    )
                image, visual_bend_angles, visual_thumb_rot_angle = compute_finger_angles_from_visual(image, results, joint_list)
                if len(visual_bend_angles) == 5:
                    ohand_target_angles[FINGER_THUMB_BEND] = translate_angle(
                        visual_bend_angles[0],
                        VISUAL_THUMB_BEND_INPUT_MIN, VISUAL_THUMB_BEND_INPUT_MAX,
                        OHAND_THUMB_BEND_RANGE[0], OHAND_THUMB_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_INDEX] = translate_angle(
                        visual_bend_angles[1],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_MIDDLE] = translate_angle(
                        visual_bend_angles[2],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_RING] = translate_angle(
                        visual_bend_angles[3],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                    ohand_target_angles[FINGER_PINKY] = translate_angle(
                        visual_bend_angles[4],
                        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX,
                        OHAND_FINGER_BEND_RANGE[0], OHAND_FINGER_BEND_RANGE[1]
                    )
                if visual_thumb_rot_angle is not None:
                    ohand_target_angles[FINGER_THUMB_ROT] = translate_angle(
                        visual_thumb_rot_angle,
                        VISUAL_THUMB_ROT_INPUT_MIN, VISUAL_THUMB_ROT_INPUT_MAX,
                        OHAND_THUMB_ROT_RANGE[0], OHAND_THUMB_ROT_RANGE[1]
                    )
                    cv2.putText(image, f"OHandThumbRotTgt: {ohand_target_angles.get(FINGER_THUMB_ROT, 0.0):.1f}",
                                (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 128, 255), 1, cv2.LINE_AA)
                y_offset = 20
                if FINGER_THUMB_BEND in ohand_target_angles:
                    cv2.putText(image, f"OHandThBend: {ohand_target_angles[FINGER_THUMB_BEND]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_INDEX in ohand_target_angles:
                    cv2.putText(image, f"OHandIdxBend: {ohand_target_angles[FINGER_INDEX]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_MIDDLE in ohand_target_angles:
                    cv2.putText(image, f"OHandMidBend: {ohand_target_angles[FINGER_MIDDLE]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_RING in ohand_target_angles:
                    cv2.putText(image, f"OHandRngBend: {ohand_target_angles[FINGER_RING]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20
                if FINGER_PINKY in ohand_target_angles:
                    cv2.putText(image, f"OHandPnkBend: {ohand_target_angles[FINGER_PINKY]:.1f}", (10, y_offset),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
                    y_offset += 20

            current_time = time.time()
            if ohand_target_angles and (current_time - last_sent_time > OHAND_SEND_INTERVAL):
                print("正在发送角度到 OHand:")
                for finger_idx, angle_val in ohand_target_angles.items():
                    print(f"  {finger_idx}: {angle_val:.2f} degrees")
                    try:
                        success = ohand.set_finger_target_angle(finger_idx, angle_val)
                        # if not success:
                        #     print(f"    设置手指 {finger_idx} 失败 (Modbus)。")
                    except Exception as e:
                        print(f"    设置手指 {finger_idx} 时发生错误: {e}")
                last_sent_time = current_time
                print("-" * 20)

            # 设置窗口大小（可选：取消注释并调整尺寸）
            # cv2.namedWindow('OHand Visual Control', cv2.WINDOW_NORMAL)
            # cv2.resizeWindow('OHand Visual Control', 1280, 720)  # 宽度, 高度
            
            cv2.imshow('OHand Visual Control', image)

            if cv2.waitKey(5) & 0xFF == ord('q'):
                print("正在退出...")
                break

    except KeyboardInterrupt:
        print("用户中断。正在退出。")
    except Exception as e:
        print(f"发生意外错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("正在清理资源...")
        if camera:
            close_connected_cameras(camera)  # 使用您相机库的正确关闭方式
        cv2.destroyAllWindows()
        if ohand and ohand.is_connected:
            print("断开连接前稍微张开手...")
            try:
                safe_open_angle = 180  # 可以调整
                mid_rot_angle = (OHAND_THUMB_ROT_RANGE[0] + OHAND_THUMB_ROT_RANGE[1]) / 2

                ohand.set_finger_target_angle(FINGER_THUMB_BEND, safe_open_angle)
                ohand.set_finger_target_angle(FINGER_INDEX, safe_open_angle)
                ohand.set_finger_target_angle(FINGER_MIDDLE, safe_open_angle)
                ohand.set_finger_target_angle(FINGER_RING, safe_open_angle)
                ohand.set_finger_target_angle(FINGER_PINKY, safe_open_angle)
                ohand.set_finger_target_angle(FINGER_THUMB_ROT, mid_rot_angle)
                time.sleep(0.5)  # 给机械手一点时间响应
            except Exception as e:
                print(f"设置 OHand 到张开位置时发生错误: {e}")
            ohand.disconnect()
        print("清理完成。")


if __name__ == '__main__':
    main()