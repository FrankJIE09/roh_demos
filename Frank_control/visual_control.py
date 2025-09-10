# -*- coding: utf-8 -*-

# 导入必要的库
import cv2
import mediapipe as mp
import numpy as np
import time
import yaml

# 移除 Orbbec 相机导入，改用 OpenCV 默认相机

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
# 移除 Orbbec 相机序列号配置


def initialize_camera():
    """初始化摄像头，尝试不同的索引和设置"""
    # 尝试不同的后端
    backends = [cv2.CAP_ANY, cv2.CAP_DSHOW, cv2.CAP_MSMF, cv2.CAP_V4L2]
    backend_names = ["ANY", "DSHOW", "MSMF", "V4L2"]
    
    for backend_idx, backend in enumerate(backends):
        print(f"尝试后端: {backend_names[backend_idx]}")
        for camera_index in [0, 1, 2]:
            print(f"  尝试打开摄像头索引 {camera_index}...")
            try:
                camera = cv2.VideoCapture(camera_index, backend)
                if camera.isOpened():
                    print(f"摄像头索引 {camera_index} 已打开，设置1920x1080分辨率...")
                    
                    # 立即尝试设置1920x1080分辨率
                    try:
                        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                        camera.set(cv2.CAP_PROP_FPS, 30)  # 提高帧率到30fps
                        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 保持最小缓冲区
                        
                        # 验证分辨率设置
                        actual_width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)
                        actual_height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
                        print(f"设置后分辨率: {actual_width}x{actual_height}")
                        
                    except Exception as e:
                        print(f"设置分辨率时出错: {e}")
                    
                    # 多次尝试读取帧以确保稳定性
                    success_count = 0
                    for attempt in range(5):
                        try:
                            ret, test_frame = camera.read()
                            if ret and test_frame is not None and test_frame.size > 0:
                                success_count += 1
                                print(f"  尝试 {attempt + 1}: 成功读取帧 {test_frame.shape}")
                            else:
                                print(f"  尝试 {attempt + 1}: 读取失败")
                        except Exception as e:
                            print(f"  尝试 {attempt + 1}: 异常 {e}")
                        
                        time.sleep(0.1)  # 短暂等待
                    
                    if success_count >= 3:  # 至少成功3次
                        print(f"摄像头索引 {camera_index} 启动成功！")
                        print(f"摄像头分辨率: {test_frame.shape[1]}x{test_frame.shape[0]}")
                        print(f"摄像头属性设置完成")
                        return camera
                    else:
                        print(f"摄像头索引 {camera_index} 读取不稳定，尝试下一个")
                        camera.release()
                else:
                    print(f"摄像头索引 {camera_index} 无法打开")
            except Exception as e:
                print(f"摄像头索引 {camera_index} 初始化错误: {e}")
                if 'camera' in locals():
                    camera.release()
    
    # 如果所有后端都失败，尝试最简单的初始化
    print("尝试最简单的摄像头初始化...")
    try:
        camera = cv2.VideoCapture(0)
        if camera.isOpened():
            # 尝试设置1920x1080
            try:
                camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                camera.set(cv2.CAP_PROP_FPS, 30)  # 提高帧率到30fps
                camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 保持最小缓冲区
                print("尝试设置1920x1080分辨率...")
            except Exception as e:
                print(f"设置分辨率时出错: {e}")
            
            ret, test_frame = camera.read()
            if ret and test_frame is not None and test_frame.size > 0:
                print(f"简单初始化成功！分辨率: {test_frame.shape[1]}x{test_frame.shape[0]}")
                return camera
            else:
                camera.release()
    except Exception as e:
        print(f"简单初始化也失败: {e}")
    
    return None


def load_config(config_path='config.yaml'):
    """加载配置文件."""
    global OHAND_SERIAL_PORT, OHAND_SLAVE_ID, \
        VISUAL_THUMB_BEND_INPUT_MIN, VISUAL_THUMB_BEND_INPUT_MAX, \
        VISUAL_OTHER_FINGERS_BEND_INPUT_MIN, VISUAL_OTHER_FINGERS_BEND_INPUT_MAX, \
        VISUAL_THUMB_ROT_INPUT_MIN, VISUAL_THUMB_ROT_INPUT_MAX, \
        OHAND_THUMB_BEND_RANGE, OHAND_FINGER_BEND_RANGE, OHAND_THUMB_ROT_RANGE, \
        MP_MIN_DETECTION_CONFIDENCE, MP_MIN_TRACKING_CONFIDENCE, \
        MP_MAX_NUM_HANDS, OHAND_SEND_INTERVAL

    try:
        with open(config_path, 'r', encoding='utf-8') as file:
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

        # 移除 Orbbec 相机配置，改用系统默认相机

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
        min_tracking_confidence=MP_MIN_TRACKING_CONFIDENCE,
        model_complexity=0)  # 使用轻量级模型，减少延迟
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

    print("正在初始化系统默认摄像头...")
    camera = initialize_camera()
    
    if camera is None:
        print("错误：无法打开任何摄像头。请检查摄像头是否连接或被其他程序占用。")
        if ohand.is_connected:
            ohand.disconnect()
        return

    print("手部追踪已启动。按 'q' 键退出。")
    last_sent_time = time.time()
    frame_error_count = 0
    max_frame_errors = 10  # 最大连续错误次数
    
    # 优化发送频率，减少延迟
    target_send_interval = 0.02  # 20ms发送间隔，50fps

    # 设置窗口为全屏，只需设置一次
    cv2.namedWindow('OHand Visual Control', cv2.WINDOW_NORMAL)
    cv2.setWindowProperty('OHand Visual Control', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while True:
            try:
                # 减少延迟，提高响应速度
                time.sleep(0.016)  # 约60fps，减少延迟
                ret, color_image = camera.read()
            except cv2.error as e:
                frame_error_count += 1
                print(f"摄像头读取错误: {e} (错误次数: {frame_error_count})")
                
                # 尝试重新初始化摄像头
                if frame_error_count >= 3:  # 降低重连阈值
                    print("摄像头读取持续失败，尝试重新连接...")
                    camera.release()
                    time.sleep(3)  # 增加等待时间
                    camera = initialize_camera()
                    if camera is None:
                        print("无法重新连接摄像头，退出程序。")
                        break
                    frame_error_count = 0
                else:
                    time.sleep(0.5)  # 增加错误间隔
                continue
                
            if not ret or color_image is None:
                frame_error_count += 1
                print(f"错误：无法从摄像头获取帧。 (错误次数: {frame_error_count})")
                if frame_error_count >= max_frame_errors:
                    print("摄像头连接丢失，尝试重新连接...")
                    camera.release()
                    time.sleep(1)
                    camera = initialize_camera()
                    if camera is None:
                        print("无法重新连接摄像头，退出程序。")
                        break
                    frame_error_count = 0
                time.sleep(0.1)
                continue

            # 验证图像数据的有效性
            if color_image.size == 0 or len(color_image.shape) != 3:
                frame_error_count += 1
                print(f"错误：获取到无效的图像数据。 (错误次数: {frame_error_count})")
                if frame_error_count >= max_frame_errors:
                    print("摄像头数据异常，尝试重新连接...")
                    camera.release()
                    time.sleep(1)
                    camera = initialize_camera()
                    if camera is None:
                        print("无法重新连接摄像头，退出程序。")
                        break
                    frame_error_count = 0
                time.sleep(0.1)
                continue

            try:
                image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
                image = cv2.flip(image, 1)
                frame_error_count = 0  # 重置错误计数
            except cv2.error as e:
                frame_error_count += 1
                print(f"图像处理错误: {e} (错误次数: {frame_error_count})")
                if frame_error_count >= max_frame_errors:
                    print("图像处理持续失败，尝试重新连接摄像头...")
                    camera.release()
                    time.sleep(1)
                    camera = initialize_camera()
                    if camera is None:
                        print("无法重新连接摄像头，退出程序。")
                        break
                    frame_error_count = 0
                time.sleep(0.1)
                continue

            try:
                image.flags.writeable = False
                results = hands.process(image)

                image.flags.writeable = True
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            except Exception as e:
                print(f"MediaPipe处理错误: {e}")
                time.sleep(0.1)
                continue

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
            if ohand_target_angles and (current_time - last_sent_time > target_send_interval):
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
            
            try:
                cv2.imshow('OHand Visual Control', image)
            except cv2.error as e:
                print(f"图像显示错误: {e}")
                time.sleep(0.1)
                continue

            if cv2.waitKey(1) & 0xFF == ord('q'):  # 减少waitKey延迟
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
            camera.release()  # 释放 OpenCV 摄像头资源
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