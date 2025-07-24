#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 文件名: dual_ohand_stepping_keyboard.py
# 描述: 使用键盘独立步进控制两个 OHand 灵巧手，支持姿态保存/加载。

import json
import os
import threading
import time
import queue
from pynput import keyboard # 需要同时处理 on_press 和 on_release
from pymodbus.exceptions import ModbusException
import traceback

# 导入 OHand Modbus 客户端类和手指常量
try:
    from ohand_modbus_client import (OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX,
                                     FINGER_MIDDLE, FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT)
    print("成功导入 'ohand_modbus_client' 模块。")
except ImportError as e:
    print(f"错误：无法导入 'ohand_modbus_client' 或其依赖 'roh_registers_v1.py'。Import Error: {e}")
    exit(1)
except Exception as e:
    print(f"导入 'ohand_modbus_client' 时发生未知错误: {e}")
    exit(1)

# --- 配置常量 ---
# *** 请根据你的实际硬件连接修改 ***
OHAND_LEFT_PORT = '/dev/ttyUSB0'     # 左手 OHand 串口
OHAND_LEFT_ID = 2                   # 左手 OHand Slave ID
OHAND_RIGHT_PORT = '/dev/ttyUSB0'    # 右手 OHand 串口
OHAND_RIGHT_ID = 2                  # 右手 OHand Slave ID (必须与左手不同!)
OHAND_BAUDRATE = 115200             # OHand 波特率

JSON_FILE = 'dual_ohand_poses_F9.json' # 保存 F1-F3 双臂位置的文件名
# OHand 逻辑位置范围
MAX_POS = 65535             # 完全闭合位置 (假设)
MIN_POS = 0                 # 完全张开位置 (假设)
CONTROL_STEP = 1000         # 每次步进的逻辑位置量 (可调整)
LOOP_SLEEP_TIME = 0.02      # 主循环和发送循环的休眠时间 (秒)，影响响应速度和 CPU 占用

# --- 全局变量 ---
ohand_left_client = OHandModbusClient(port=OHAND_LEFT_PORT, slave_id=OHAND_LEFT_ID)
ohand_right_client = OHandModbusClient(port=OHAND_RIGHT_PORT, slave_id=OHAND_RIGHT_ID)
ohand_left_connected = False
ohand_right_connected = False
listener_active = True
saved_positions = {}        # 内存中存储 F1-F3 的双臂位置
command_queue = queue.Queue() # 线程安全的命令队列

# --- NEW: 存储当前按下的控制键 ---
# 使用集合存储当前按下的有效控制键字符 (小写)
pressed_control_keys = set()
# 使用锁来保护对 pressed_control_keys 的并发访问 (虽然此简单场景下可能不是严格必需)
pressed_keys_lock = threading.Lock()
# --- NEW: 存储每个手指的目标位置 (由步进控制更新) ---
# 初始化为张开状态
current_target_positions = {
    "left": [MIN_POS] * 6,
    "right": [MIN_POS] * 6
}
target_pos_lock = threading.Lock() # 保护目标位置字典的访问

# --- 按键映射 (改为控制方向) ---
# key -> (hand_client_ref, finger_index, direction)
# direction: -1 表示减少位置值 (张开), +1 表示增加位置值 (闭合)
#           对于拇指旋转，方向的意义取决于你的 MIN/MAX 定义

# 左手控制键 (示例: QAZ WSX EDC RFV TG B YHN?) - 需要根据键盘布局调整
# QAZ: 拇指弯曲 张开/闭合
# WSX: 食指 张开/闭合
# ...
left_control_map = {
    'q': ("left", FINGER_THUMB_BEND, -1), 'a': ("left", FINGER_THUMB_BEND, +1),
    'w': ("left", FINGER_INDEX,      -1), 's': ("left", FINGER_INDEX,      +1),
    'e': ("left", FINGER_MIDDLE,     -1), 'd': ("left", FINGER_MIDDLE,     +1),
    'r': ("left", FINGER_RING,       -1), 'f': ("left", FINGER_RING,       +1),
    't': ("left", FINGER_PINKY,      -1), 'g': ("left", FINGER_PINKY,      +1),
    'y': ("left", FINGER_THUMB_ROT,  -1), 'h': ("left", FINGER_THUMB_ROT,  +1), # 假设 -1 -> MIN, +1 -> MAX
}

# 右手控制键 (示例: UIK OJL P;[ ']) - 需要根据键盘布局调整
# UIK: 拇指弯曲 张开/闭合
# OJL: 食指 张开/闭合
# ...
right_control_map = {
    'u': ("right", FINGER_THUMB_BEND, -1), 'j': ("right", FINGER_THUMB_BEND, +1),
    'i': ("right", FINGER_INDEX,      -1), 'k': ("right", FINGER_INDEX,      +1),
    'o': ("right", FINGER_MIDDLE,     -1), 'l': ("right", FINGER_MIDDLE,     +1),
    'p': ("right", FINGER_RING,       -1), ';': ("right", FINGER_RING,       +1), # ';' 可能需要特殊处理或更换
    '[': ("right", FINGER_PINKY,      -1), "'": ("right", FINGER_PINKY,      +1), # '[' 和 ''' 可能需要特殊处理或更换
    ']': ("right", FINGER_THUMB_ROT,  -1), '\\': ("right", FINGER_THUMB_ROT,  +1), # ']' 和 '\' 可能需要特殊处理或更换
}

# 合并所有控制键映射
all_control_keys = set(left_control_map.keys()) | set(right_control_map.keys())

# 功能键映射 (F1-F3 用于保存)
# 2) 扩展保存键映射：F1‑F9   （enumerate 改成手动列出最直观）
save_key_map = {
    keyboard.Key.f1: "F1",
    keyboard.Key.f2: "F2",
    keyboard.Key.f3: "F3",
    keyboard.Key.f4: "F4",
    keyboard.Key.f5: "F5",
    keyboard.Key.f6: "F6",
    keyboard.Key.f7: "F7",
    keyboard.Key.f8: "F8",
    keyboard.Key.f9: "F9",
}# 加载键映射 (数字键 1-3)
load_key_map = {str(i): f"F{i}" for i in range(1, 10)}


# --- 功能函数 (与之前基本相同) ---
def load_positions_from_file():
    """从 JSON 文件加载已保存的双手臂位置到内存。"""
    global saved_positions
    if os.path.exists(JSON_FILE):
        try:
            with open(JSON_FILE, 'r', encoding='utf-8') as f: saved_positions = json.load(f)
            print(f"Loaded {len(saved_positions)} saved poses from '{JSON_FILE}'.")
        except Exception as e: print(f"Error loading poses from '{JSON_FILE}': {e}"); saved_positions = {}
    else: print(f"Pose file '{JSON_FILE}' not found, starting fresh."); saved_positions = {}

def save_positions_to_file():
    """将内存中的双手臂位置保存到 JSON 文件。"""
    global saved_positions
    try:
        with open(JSON_FILE, 'w', encoding='utf-8') as f: json.dump(saved_positions, f, indent=4)
    except Exception as e: print(f"Error saving poses to '{JSON_FILE}': {e}")

def get_current_hand_positions(client: OHandModbusClient):
    """辅助函数：获取单个手的所有手指当前位置。"""
    if not (client and client.is_connected): return None
    current_pos = {}
    try:
        for i in range(6):
            pos = client.get_finger_current_pos(i)
            if pos is None: print(f"Warning: Failed to read pos for finger {i} on {client.client.port}."); return None
            current_pos[str(i)] = pos
        return current_pos
    except Exception as e: print(f"Error reading positions from {client.client.port}: {e}"); return None

def save_current_positions(slot_name):
    """获取当前左右两个手的所有手指位置并保存到指定槽位。"""
    global ohand_left_client, ohand_right_client, saved_positions
    print(f"Attempting to save current dual pose to slot '{slot_name}'...")
    left_positions = get_current_hand_positions(ohand_left_client)
    right_positions = get_current_hand_positions(ohand_right_client)
    if left_positions is not None and right_positions is not None:
        saved_positions[slot_name] = {"left": left_positions, "right": right_positions}
        save_positions_to_file()
        print(f"Dual pose saved successfully to slot '{slot_name}'.")
    else: print("Error: Failed to read positions from one or both hands, cannot save.")

def load_and_set_positions(slot_name):
    """从指定槽位加载左右手位置并命令 OHand 移动。"""
    global ohand_left_client, ohand_right_client, saved_positions, current_target_positions, target_pos_lock
    print(f"Attempting to load and set dual pose from slot '{slot_name}'...")
    if slot_name not in saved_positions: print(f"Error: Slot '{slot_name}' not found in saved poses."); return
    slot_data = saved_positions[slot_name]
    left_positions_dict = slot_data.get("left"); right_positions_dict = slot_data.get("right")

    new_target_left = current_target_positions["left"][:] # Start with current target
    new_target_right = current_target_positions["right"][:]
    loaded_left = False
    loaded_right = False

    # Process Left Hand
    if left_positions_dict:
        temp_list = [0] * 6; valid = True
        for i in range(6):
            key = str(i); val = left_positions_dict.get(key)
            if val is not None: temp_list[i] = val
            else: print(f"Warning: Missing left finger {i} data in slot '{slot_name}'."); valid = False; break
        if valid: new_target_left = temp_list; loaded_left = True
    # Process Right Hand
    if right_positions_dict:
        temp_list = [0] * 6; valid = True
        for i in range(6):
            key = str(i); val = right_positions_dict.get(key)
            if val is not None: temp_list[i] = val
            else: print(f"Warning: Missing right finger {i} data in slot '{slot_name}'."); valid = False; break
        if valid: new_target_right = temp_list; loaded_right = True

    # Update global target positions atomically (if loaded)
    with target_pos_lock:
        if loaded_left: current_target_positions["left"] = new_target_left
        if loaded_right: current_target_positions["right"] = new_target_right

    print(f"Target positions updated from slot '{slot_name}'. Sending thread will move the hands.")


# --- NEW: 键盘监听回调 (处理按下和释放) ---

def on_press(key):
    """处理按键按下事件。"""
    global listener_active, command_queue, pressed_keys_lock, pressed_control_keys

    char = None
    try: char = getattr(key, 'char', None)
    except AttributeError: pass

    # 处理功能键 (Save/Load/Exit) -> 使用队列
    if key == keyboard.Key.esc: command_queue.put(("exit", None)); return False # Exit on Esc press
    elif key in save_key_map: command_queue.put(("save", save_key_map[key]))
    elif char and char.lower() in load_key_map: command_queue.put(("load", load_key_map[char.lower()]))
    # 处理控制键 -> 更新按下状态集合
    elif char and char.lower() in all_control_keys:
        with pressed_keys_lock:
            pressed_control_keys.add(char.lower())
            # print(f"Pressed: {char.lower()}, Current: {pressed_control_keys}") # 调试

def on_release(key):
    """处理按键释放事件。"""
    global pressed_keys_lock, pressed_control_keys

    char = None
    try: char = getattr(key, 'char', None)
    except AttributeError: pass

    # 如果是控制键，从集合中移除
    if char and char.lower() in all_control_keys:
        with pressed_keys_lock:
            pressed_control_keys.discard(char.lower()) # 使用 discard 避免 key 不存在时出错
            # print(f"Released: {char.lower()}, Current: {pressed_control_keys}") # 调试

    # 监听器退出条件由 on_press 中的 Esc 或 listener_active 标志控制

def keyboard_listener_thread():
    """运行键盘监听器的线程函数。"""
    global listener_active
    print("启动键盘监听器...")
    try:
        # 同时监听按下和释放事件
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join() # 等待监听器停止 (由 on_press 返回 False 触发)
    except Exception as e:
        print(f"键盘监听器线程出错: {e}")
        traceback.print_exc()
        if listener_active: command_queue.put(("exit", None)) # 尝试通知主线程退出
    finally:
        print("键盘监听器已停止。")


# --- NEW: 独立线程用于发送 Modbus 命令 ---
def command_sender_thread():
    """
    独立线程，定期检查目标位置并向 OHand 发送指令，
    避免阻塞主循环和键盘监听。
    """
    global listener_active, current_target_positions, target_pos_lock
    global ohand_left_client, ohand_right_client
    global ohand_left_connected, ohand_right_connected

    # 记录上次发送给每个手指的位置，以减少不必要的重复发送
    last_sent_positions = {
        "left": [None] * 6,
        "right": [None] * 6
    }

    finger_indices = [
        FINGER_THUMB_BEND, FINGER_INDEX, FINGER_MIDDLE,
        FINGER_RING, FINGER_PINKY, FINGER_THUMB_ROT
    ]

    while listener_active:
        # 获取当前目标位置的副本
        with target_pos_lock:
            target_left = current_target_positions["left"][:]
            target_right = current_target_positions["right"][:]

        # --- 发送左手指令 ---
        if ohand_left_connected and ohand_left_client:
            for i, finger_idx in enumerate(finger_indices):
                target_pos_int = int(round(target_left[i]))
                # 只有当目标位置与上次发送不同时才发送
                if target_pos_int != last_sent_positions["left"][i]:
                    try:
                        # print(f"Sending Left F{finger_idx} -> {target_pos_int}") # 调试
                        if ohand_left_client.set_finger_target_pos(finger_idx, target_pos_int):
                            last_sent_positions["left"][i] = target_pos_int # 更新已发送记录
                        else:
                             print(f"Error sending Left F{finger_idx}")
                             last_sent_positions["left"][i] = None # 发送失败，下次重试
                    except Exception as e:
                        print(f"Error sending Left F{finger_idx}: {e}")
                        last_sent_positions["left"][i] = None # 发送失败，下次重试

        # --- 发送右手指令 ---
        if ohand_right_connected and ohand_right_client:
             for i, finger_idx in enumerate(finger_indices):
                target_pos_int = int(round(target_right[i]))
                if target_pos_int != last_sent_positions["right"][i]:
                    try:
                        # print(f"Sending Right F{finger_idx} -> {target_pos_int}") # 调试
                        if ohand_right_client.set_finger_target_pos(finger_idx, target_pos_int):
                            last_sent_positions["right"][i] = target_pos_int
                        else:
                            print(f"Error sending Right F{finger_idx}")
                            last_sent_positions["right"][i] = None
                    except Exception as e:
                        print(f"Error sending Right F{finger_idx}: {e}")
                        last_sent_positions["right"][i] = None

        # 控制发送频率
        time.sleep(LOOP_SLEEP_TIME)

    print("命令发送线程结束。")


# --- 主程序 ---
if __name__ == "__main__":
    print("--- OHand 双臂独立步进键盘控制器 ---")
    print("控制按键:")
    print("  左手: Q/A(拇指弯±) W/S(食指±) E/D(中指±) R/F(无名指±) T/G(小指±) Y/H(拇指旋±)")
    print("  右手: U/J(拇指弯±) I/K(食指±) O/L(中指±) P/;(无名指±) [/'(小指±) ]/\(拇指旋±)")
    print("        (上面某些右手键可能需要调整)")
    print("功能按键:")
    print("  F1‑F9: 保存当前【双臂】所有手指位置")
    print("  1‑9:   加载并移动【双臂】到 F1‑F9 保存的位置")
    print("  Esc:   退出程序")
    print("-" * 30)

    # 加载保存的位置
    load_positions_from_file()

    # --- 连接 OHand ---
    # (与上个版本相同的连接逻辑)
    if ohand_left_client:
        if ohand_left_client.connect(): ohand_left_connected = True; print("Left OHand connected.")
        else: print(f"Error connecting Left OHand ({OHAND_LEFT_PORT}).")
    if ohand_right_client:
        if ohand_left_client and ohand_left_client.slave_id == ohand_right_client.slave_id: # Check ID again
             print(f"Warning: Left and Right OHand Slave IDs ({ohand_left_client.slave_id}) are the same!")
        if ohand_right_client.connect(): ohand_right_connected = True; print("Right OHand connected.")
        else: print(f"Error connecting Right OHand ({OHAND_RIGHT_PORT}).")

    if not (ohand_left_connected or ohand_right_connected): print("Error: No OHand connected. Exiting."); exit(1)
    elif not ohand_left_connected: print("Warning: Only Right OHand connected.")
    elif not ohand_right_connected: print("Warning: Only Left OHand connected.")


    # --- 主循环 - 现在处理按键状态和队列命令 ---
    try:
        # 启动键盘监听线程
        listener_thread = threading.Thread(target=keyboard_listener_thread, daemon=True)
        listener_thread.start()

        # --- NEW: 启动命令发送线程 ---
        sender_thread = threading.Thread(target=command_sender_thread, daemon=True)
        sender_thread.start()

        print("\n控制器已启动，按 Esc 退出。")

        while listener_active:
            # --- 处理步进移动 ---
            with pressed_keys_lock:
                active_keys = pressed_control_keys.copy() # 获取当前按下键的快照

            if active_keys: # 只有当有键按下时才进行处理
                with target_pos_lock: # 获取锁以修改目标位置
                    new_target_left = current_target_positions["left"][:]
                    new_target_right = current_target_positions["right"][:]

                    for key in active_keys:
                        if key in left_control_map:
                            _, finger_idx, direction = left_control_map[key]
                            new_target_left[finger_idx] += direction * CONTROL_STEP
                            new_target_left[finger_idx] = max(MIN_POS, min(new_target_left[finger_idx], MAX_POS))
                        elif key in right_control_map:
                            _, finger_idx, direction = right_control_map[key]
                            new_target_right[finger_idx] += direction * CONTROL_STEP
                            new_target_right[finger_idx] = max(MIN_POS, min(new_target_right[finger_idx], MAX_POS))

                    # 更新全局目标位置
                    current_target_positions["left"] = new_target_left
                    current_target_positions["right"] = new_target_right

            # --- 处理队列中的功能命令 (Save/Load/Exit) ---
            try:
                cmd, value = command_queue.get_nowait() # 非阻塞获取

                if cmd == "save": save_current_positions(value)
                elif cmd == "load": load_and_set_positions(value)
                elif cmd == "exit": print("正在退出主循环..."); listener_active = False; break

            except queue.Empty:
                pass # 队列为空，正常
            except Exception as e:
                print(f"主循环处理功能命令时出错: {e}")
                traceback.print_exc()

            # 控制主循环频率
            time.sleep(LOOP_SLEEP_TIME) # 主循环也需要休眠

    except KeyboardInterrupt: print("\n检测到 Ctrl+C，正在退出..."); listener_active = False
    except Exception as e: print(f"发生未处理的异常: {e}"); traceback.print_exc()
    finally:
        print("正在进行清理...")
        listener_active = False # 确保其他线程能退出

        # 等待线程结束
        if 'listener_thread' in locals() and listener_thread.is_alive():
            print("等待键盘监听器结束...")
            listener_thread.join(timeout=1.0)
        if 'sender_thread' in locals() and sender_thread.is_alive():
            print("等待命令发送器结束...")
            sender_thread.join(timeout=1.0) # 给点时间完成最后的发送

        # 断开 OHand 连接
        if ohand_left_client and ohand_left_client.is_connected: print("断开左手 OHand..."); ohand_left_client.disconnect()
        if ohand_right_client and ohand_right_client.is_connected: print("断开右手 OHand..."); ohand_right_client.disconnect()

        print("程序已退出。")