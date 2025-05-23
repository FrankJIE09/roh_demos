# 文件名: dual_ohand_visual_control_v2.py
# 描述: 使用单个 Orbbec 相机同时检测左右手，并分别控制两个 OHand 灵巧手。
#       修改为使用单个 Modbus 客户端实例控制两个不同 ID 的 OHand。

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
    # 确保 ohand_modbus_client.py 文件已经根据前面的说明进行了修改
    from ohand_modbus_client_one_usb import (OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX,
                                     FINGER_MIDDLE, FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT)
    print("成功导入 'ohand_modbus_client' 模块。")
    print("提示：请务必确保 'ohand_modbus_client.py' 已更新以支持多从站控制 (target_slave_id 参数)。")
except ImportError as e:
    print(f"错误：无法导入 'ohand_modbus_client' 或其依赖。Import Error: {e}")
    exit(1)
except Exception as e:
    print(f"导入 'ohand_modbus_client' 时发生未知错误: {e}")
    exit(1)


# --- 常量与配置 ---
# Orbbec 相机配置
ORBBEC_CAMERA_SN = None # 设置为 None 将使用找到的第一个设备，或指定序列号

# OHand Modbus 配置
# 注意：现在只使用一个物理串口，但通过不同的ID控制两个OHand
OHAND_COMMON_PORT = '/dev/ttyUSB0'   # *** 所有 OHand 连接的共享串口 ***
OHAND_LEFT_ID = 2                   # *** 左手 OHand 的 Modbus Slave ID ***
OHAND_RIGHT_ID = 3                  # *** 右手 OHand 的 Modbus Slave ID (必须与左手不同) ***
OHAND_BAUDRATE = 115200             # OHand 的 Modbus 波特率

# OHand 逻辑位置范围
OHAND_POS_OPEN = 0       # 手指完全张开时的逻辑位置值
OHAND_POS_CLOSED = 65535  # 手指完全闭合时的逻辑位置值
OHAND_THUMB_ROT_MIN = 65535     # 拇指旋转的一个极限逻辑位置值 (对应手掌远离，外展)
OHAND_THUMB_ROT_MAX = 0 # 拇指旋转的另一个极限逻辑位置值 (对应手掌靠近，内收)

# --- 输入参数范围估计 (*** 需要通过观察和调试精确校准! ***) ---
# 这些值强烈依赖于你的手和相机距离，务必调试！
BEND_DIST_MIN = 0.15  # 手指非常弯曲时的距离 (例如：指尖到手腕，非常靠近)
BEND_DIST_MAX = 0.25  # 手指伸直时的距离 (例如：指尖到手腕，较远)

THUMB_ROT_DIST_MIN = 0.08  # 拇指靠近手掌（内收）时的距离 (例如：拇指尖到小指掌指关节，很近)
THUMB_ROT_DIST_MAX = 0.15  # 拇指外展（远离手掌）时的距离 (例如：拇指尖到小指掌指关节，较远)


# 其他常量
SEND_INTERVAL = 0.05        # 控制命令发送的最小时间间隔（秒）
POS_CHANGE_THRESHOLD = 500  # 位置变化阈值，小于此值不发送（减少通信量）
ESC_KEY = 27                # ESC 键的 ASCII 码

# --- MediaPipe Hands 初始化 (检测双手) ---
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    model_complexity=0, # 0 或 1，1 更准确但速度慢
    max_num_hands=2,    # 最大检测手部数量
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7)
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# --- OHand Modbus 客户端 初始化 (现在只初始化一个客户端) ---
ohand_common_client: OHandModbusClient | None = None
try:
    print(f"初始化 OHand Modbus (Port: {OHAND_COMMON_PORT}, 初始ID: {OHAND_LEFT_ID})")
    # 初始ID可以是任意一个，因为我们会在发送时指定
    ohand_common_client = OHandModbusClient(port=OHAND_COMMON_PORT, slave_id=OHAND_LEFT_ID, baudrate=OHAND_BAUDRATE)
except Exception as e:
    print(f"创建 OHandModbusClient 失败: {e}")

# --- 工具函数 ---
def map_value(value, in_min, in_max, out_min, out_max):
    """将一个值从输入范围线性映射到输出范围"""
    if in_max == in_min: return out_min
    # 确保 value 在输入范围内
    value = max(in_min, min(value, in_max))
    mapped = (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    # 确保映射结果在输出范围内
    if out_min <= out_max: return max(min(mapped, out_max), out_min)
    else: return max(min(mapped, out_min), out_max)

def calculate_distance(p1, p2):
    """计算两个 MediaPipe landmark 点之间的欧氏距离 (3D)"""
    if p1 is None or p2 is None: return float('inf')
    return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)

# --- OHand 控制函数 (现在只传入一个客户端实例，并指定ID) ---
def send_to_ohand_individual(client: OHandModbusClient | None, target_positions: list, slave_id: int):
    """
    使用指定的 OHand Modbus 客户端为 6 个自由度发送目标位置指令。
    :param client: OHand Modbus 客户端实例。
    :param target_positions: 包含 6 个目标位置的列表。
    :param slave_id: 目标 OHand 的 Modbus Slave ID。
    """
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
            # target_pos_int 已在 set_finger_target_pos 内部限制，但这里可以再做一次防御性检查
            target_pos_int = max(0, min(target_pos_int, 65535))
            # 调用修改后的 set_finger_target_pos，传入 slave_id
            client.set_finger_target_pos(finger_index, target_pos_int, target_slave_id=slave_id)
    except Exception as e:
        # 确保 client.port 是可访问的
        port_info = client.client.port if hasattr(client, 'client') and hasattr(client.client, 'port') else 'N/A'
        print(f"发送 OHand 命令时出错 (客户端端口: {port_info}, ID: {slave_id}): {e}")

# --- 主执行逻辑 ---
def main():
    if ohand_common_client is None:
        print("错误：OHand Modbus 客户端未能成功初始化，程序退出。")
        return

    orbbec_camera = None
    ohand_connected = False # 只有一个连接状态了

    try:
        # --- 初始化 Orbbec 相机 ---
        print("查找奥比中光设备...")
        available_sns = get_serial_numbers()
        if not available_sns: print("错误：未找到奥比中光设备。"); return
        camera_sn = ORBBEC_CAMERA_SN if ORBBEC_CAMERA_SN else available_sns[0]
        print(f"尝试初始化 Orbbec 相机，序列号: {camera_sn if camera_sn else '第一个可用设备'}")
        orbbec_camera = OrbbecCamera(camera_sn if camera_sn else available_sns[0])
        print(f"Orbbec 相机 {orbbec_camera.get_serial_number()} 初始化成功。")

        # --- 连接 OHand 设备 (现在只连接一次共享的客户端) ---
        if ohand_common_client:
            print(f"连接 OHand Modbus (Port: {OHAND_COMMON_PORT})...")
            if ohand_common_client.connect():
                ohand_connected = True
                print(f"OHand Modbus 客户端 ({OHAND_COMMON_PORT}) 连接成功。")
                # 尝试与两个ID通信，确认它们都在线上
                print(f"尝试与 OHand ID={OHAND_LEFT_ID} 通信...")
                try:
                    # 读取一个寄存器作为心跳，或者发送一个简单的归零命令
                    ohand_common_client.set_finger_target_pos(FINGER_THUMB_BEND, OHAND_POS_OPEN, OHAND_LEFT_ID)
                    print(f"OHand ID={OHAND_LEFT_ID} 响应正常。")
                except Exception as e:
                    print(f"警告: OHand ID={OHAND_LEFT_ID} 未响应或连接失败: {e}")

                print(f"尝试与 OHand ID={OHAND_RIGHT_ID} 通信...")
                try:
                    ohand_common_client.set_finger_target_pos(FINGER_THUMB_BEND, OHAND_POS_OPEN, OHAND_RIGHT_ID)
                    print(f"OHand ID={OHAND_RIGHT_ID} 响应正常。")
                except Exception as e:
                    print(f"警告: OHand ID={OHAND_RIGHT_ID} 未响应或连接失败: {e}")

            else:
                print(f"错误：连接 OHand Modbus 客户端 ({OHAND_COMMON_PORT}) 失败。")

        if not ohand_connected:
            print("警告：OHand 灵巧手 Modbus 连接失败，将仅进行视觉追踪。")

        # --- 启动 Orbbec 数据流 ---
        print("启动奥比中光相机数据流 (彩色+深度)...")
        # 确保深度流也开启，因为 calculate_distance 使用了 3D 坐标
        # use_alignment=True 和 enable_sync=True 推荐用于同步和对齐彩色与深度数据
        orbbec_camera.start_stream(color_stream=True, depth_stream=True, use_alignment=True, enable_sync=True)
        print("Orbbec 数据流已启动。")

        # --- 循环变量初始化 ---
        last_send_time = time.time()
        last_sent_positions_left = [OHAND_POS_OPEN] * 6
        last_sent_positions_right = [OHAND_POS_OPEN] * 6
        frame_counter = 0
        print("进入主循环 (在 OpenCV 窗口中按 'q' 退出)...")

        while True:
            frame_counter += 1
            # --- 1. 获取图像 ---
            # 确保获取深度图像，即使不直接显示，也用于 3D 坐标计算
            color_image, depth_image, _ = orbbec_camera.get_frames()
            if color_image is None or depth_image is None: time.sleep(0.01); continue
            if not isinstance(color_image, np.ndarray) or color_image.dtype != np.uint8 or len(color_image.shape) != 3:
                 print(f"警告：彩色图像格式不正确。"); continue

            # --- 2. MediaPipe 手部检测 (双手) ---
            image_height, image_width, _ = color_image.shape
            # MediaPipe Hands 期望 RGB 图像
            image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image_rgb.flags.writeable = False # 提高性能
            results = hands.process(image_rgb)
            color_image.flags.writeable = True # 允许在原始图像上绘图

            # 初始化当前帧的左右手目标位置
            current_target_positions_left = last_sent_positions_left[:]
            current_target_positions_right = last_sent_positions_right[:]
            detected_left_this_frame = False
            detected_right_this_frame = False

            # --- 3. 提取参数并映射 (区分左右手) ---
            if results.multi_hand_landmarks and results.multi_handedness:
                for i, hand_landmarks in enumerate(results.multi_hand_landmarks):
                    handedness = results.multi_handedness[i].classification[0].label # 'Left' or 'Right'
                    score = results.multi_handedness[i].classification[0].score

                    # 绘制手部骨架
                    mp_drawing.draw_landmarks(color_image, hand_landmarks, mp_hands.HAND_CONNECTIONS,
                                              mp_drawing_styles.get_default_hand_landmarks_style(),
                                              mp_drawing_styles.get_default_hand_connections_style())

                    # --- 确定显示的标签和颜色 ---
                    display_label = ""
                    display_color = (255, 255, 255) # 默认白色

                    # MediaPipe 识别的左右手标签是基于摄像头视角，通常是镜像的。
                    # 如果你的摄像头是正对人手，MediaPipe 识别为 'Left' (相机右侧的手) 实际上是你的右手。
                    # 因此，我们反转控制逻辑。
                    if handedness == 'Left': # MediaPipe 识别为左手 (屏幕上右侧，通常是物理右手)
                        display_label = "CONTROLLING_RIGHT_OHAND"
                        display_color = (0, 255, 0) # 右手 OHand 用绿色
                    elif handedness == 'Right': # MediaPipe 识别为右手 (屏幕上左侧，通常是物理左手)
                        display_label = "CONTROLLING_LEFT_OHAND"
                        display_color = (255, 0, 255) # 左手 OHand 用洋红
                    else:
                        continue # 如果不是左右手，跳过

                    # 在手上标注
                    # 获取手腕点的屏幕坐标进行标注
                    wrist_x = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * image_width)
                    wrist_y = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * image_height)
                    cv2.putText(color_image, f"{display_label} ({score:.2f})",
                                (wrist_x - 10, wrist_y - 20), # 稍微向上偏移
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, display_color, 2, cv2.LINE_AA)


                    # --- 获取 3D 坐标用于距离计算 ---
                    landmarks = hand_landmarks.landmark
                    wrist = landmarks[mp_hands.HandLandmark.WRIST] # 手腕
                    # MediaPipe 的 z 坐标是相对于手腕的相对深度

                    # --- 计算弯曲距离 (指尖到手腕的距离) ---
                    distances_bend = {
                        FINGER_THUMB_BEND: calculate_distance(landmarks[mp_hands.HandLandmark.THUMB_TIP], wrist),
                        FINGER_INDEX:      calculate_distance(landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP], wrist),
                        FINGER_MIDDLE:     calculate_distance(landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP], wrist),
                        FINGER_RING:       calculate_distance(landmarks[mp_hands.HandLandmark.RING_FINGER_TIP], wrist),
                        FINGER_PINKY:      calculate_distance(landmarks[mp_hands.HandLandmark.PINKY_TIP], wrist)
                    }

                    # --- 计算拇指旋转距离 (拇指尖到小指掌指关节的距离) ---
                    # 这是一个判断拇指内收/外展的常用方式
                    thumb_rot_dist = calculate_distance(landmarks[mp_hands.HandLandmark.THUMB_TIP],
                                                        landmarks[mp_hands.HandLandmark.PINKY_MCP])

                    current_hand_targets = [OHAND_POS_OPEN] * 6 # 初始化当前手势的所有目标值

                    # --- 映射弯曲距离 ---
                    for finger_ohand_idx, dist in distances_bend.items():
                        if dist != float('inf'):
                            # 距离越小（弯曲），OHand位置越大（闭合）
                            mapped_pos = map_value(dist, BEND_DIST_MIN, BEND_DIST_MAX, OHAND_POS_CLOSED, OHAND_POS_OPEN)
                            current_hand_targets[finger_ohand_idx] = mapped_pos
                        else:
                            # 如果距离无法计算，保持为张开状态
                            current_hand_targets[finger_ohand_idx] = OHAND_POS_OPEN

                    # --- 映射拇指旋转距离 ---
                    if thumb_rot_dist != float('inf'):
                        # 距离越小（内收），OHand拇指旋转位置越大（例如 THUMB_ROT_MIN）
                        mapped_rot_pos = map_value(thumb_rot_dist, THUMB_ROT_DIST_MIN, THUMB_ROT_DIST_MAX, OHAND_THUMB_ROT_MIN, OHAND_THUMB_ROT_MAX)
                        current_hand_targets[FINGER_THUMB_ROT] = mapped_rot_pos
                    else:
                        current_hand_targets[FINGER_THUMB_ROT] = OHAND_THUMB_ROT_MIN # 默认值

                    # --- 更新对应手的目标列表 ---
                    # MediaPipe 识别为 'Left' (屏幕右侧) -> 控制右手 OHand (ID=OHAND_RIGHT_ID)
                    if handedness == 'Left':
                        current_target_positions_right = current_hand_targets[:]
                        detected_right_this_frame = True
                    # MediaPipe 识别为 'Right' (屏幕左侧) -> 控制左手 OHand (ID=OHAND_LEFT_ID)
                    elif handedness == 'Right':
                        current_target_positions_left = current_hand_targets[:]
                        detected_left_this_frame = True

            # --- 4. 发送指令到 OHand ---
            current_time = time.time()
            time_elapsed = current_time - last_send_time
            should_send = time_elapsed > SEND_INTERVAL

            left_pos_changed = False
            if detected_left_this_frame:
                for i in range(6):
                    # 仅当检测到手且位置变化超过阈值时才标记为变化
                    if abs(round(current_target_positions_left[i]) - round(last_sent_positions_left[i])) > POS_CHANGE_THRESHOLD:
                        left_pos_changed = True; break
            right_pos_changed = False
            if detected_right_this_frame:
                 for i in range(6):
                    # 仅当检测到手且位置变化超过阈值时才标记为变化
                    if abs(round(current_target_positions_right[i]) - round(last_sent_positions_right[i])) > POS_CHANGE_THRESHOLD:
                        right_pos_changed = True; break

            # 只有当达到发送间隔 或 任一目标手爪位置变化超过阈值时才发送
            if ohand_connected and (should_send or left_pos_changed or right_pos_changed):
                if detected_left_this_frame:
                    send_to_ohand_individual(ohand_common_client, current_target_positions_left, OHAND_LEFT_ID)
                    last_sent_positions_left = current_target_positions_left[:] # 更新已发送位置
                if detected_right_this_frame:
                    send_to_ohand_individual(ohand_common_client, current_target_positions_right, OHAND_RIGHT_ID)
                    last_sent_positions_right = current_target_positions_right[:] # 更新已发送位置

                last_send_time = current_time # 重置发送时间

            # --- 5. 显示图像 ---
            cv2.imshow('Orbbec OHand Visual Control', color_image) # 窗口标题更具体

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
        if ohand_common_client is not None and ohand_common_client.is_connected:
            print("断开 OHand Modbus 连接...")
            try: ohand_common_client.disconnect()
            except Exception as e: print(f"断开 OHand Modbus 时出错: {e}")
        cv2.destroyAllWindows()
        print("OpenCV 窗口已关闭。")
        print("程序结束。")

# --- 程序入口 ---
if __name__ == "__main__":
    main()