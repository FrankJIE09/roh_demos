#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import os
import threading
import time
import queue
from pynput import keyboard
from ohand_modbus_client import OHandModbusClient, FINGER_THUMB_BEND, FINGER_INDEX, FINGER_MIDDLE, FINGER_RING, \
    FINGER_PINKY, FINGER_THUMB_ROT
from pymodbus.exceptions import ModbusException
# 导入你的 OHand Modbus 客户端类

# --- 配置常量 ---
SERIAL_PORT = '/dev/ttyUSB0'  # 修改为你的实际串口
#SERIAL_PORT = 'COM3'       # Windows 示例
SLAVE_ID = 2             # OHand 的 Modbus 从站地址
JSON_FILE = 'ohand_positions_f9.json' # 保存位置的文件名 (更新文件名以示区别)
CONTROL_STEP = 1000         # 每次按键移动的逻辑位置步长 (0-65535)
MAX_POS = 65535             # 最大逻辑位置
MIN_POS = 0                 # 最小逻辑位置

# --- 全局变量 ---
hand_client = None          # OHand 客户端实例
command_queue = queue.Queue() # 线程安全的命令队列
listener_active = True      # 控制监听器线程
saved_positions = {}        # 内存中存储的已保存位置

# --- 按键映射 ---
# 控制按键: key -> (finger_index, direction) direction 1 for up, -1 for down
control_key_map = {
    'q': (FINGER_THUMB_BEND, 1), 'a': (FINGER_THUMB_BEND, -1),
    'w': (FINGER_INDEX, 1),      's': (FINGER_INDEX, -1),
    'e': (FINGER_MIDDLE, 1),     'd': (FINGER_MIDDLE, -1),
    'r': (FINGER_RING, 1),       'f': (FINGER_RING, -1),
    't': (FINGER_PINKY, 1),      'g': (FINGER_PINKY, -1),
    'y': (FINGER_THUMB_ROT, 1),  'h': (FINGER_THUMB_ROT, -1),
}

# 功能键映射: key -> slot_name
# *** 更新：支持 F1 到 F9 ***
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
    # 如果需要，可以继续添加 F10, F11, F12
}

# *** 更新：支持数字键 1 到 9 ***
load_key_map = {
    '1': "F1",
    '2': "F2",
    '3': "F3",
    '4': "F4",
    '5': "F5",
    '6': "F6",
    '7': "F7",
    '8': "F8",
    '9': "F9",
}

# --- 功能函数 (与之前版本相同) ---

def load_positions_from_file():
    """从 JSON 文件加载已保存的位置到内存。"""
    global saved_positions
    if os.path.exists(JSON_FILE):
        try:
            with open(JSON_FILE, 'r') as f:
                saved_positions = json.load(f)
            print(f"已从 '{JSON_FILE}' 加载 {len(saved_positions)} 个位置。")
        except Exception as e:
            print(f"错误：加载位置文件 '{JSON_FILE}' 失败: {e}")
            saved_positions = {}
    else:
        print(f"位置文件 '{JSON_FILE}' 不存在，将创建一个新的。")
        saved_positions = {}

def save_positions_to_file():
    """将内存中的位置保存到 JSON 文件。"""
    global saved_positions
    try:
        with open(JSON_FILE, 'w') as f:
            json.dump(saved_positions, f, indent=4)
    except Exception as e:
        print(f"错误：保存位置到文件 '{JSON_FILE}' 失败: {e}")

def save_current_positions(slot_name):
    """获取当前所有手指位置并保存到指定槽位。"""
    global hand_client, saved_positions
    if not hand_client or not hand_client.is_connected:
        print("错误：无法保存位置，未连接到 OHand。")
        return

    print(f"尝试保存当前位置到槽位 '{slot_name}'...")
    current_pos = {}
    try:
        for i in range(6): # 0 到 5
            pos = hand_client.get_finger_current_pos(i)
            if pos is None:
                print(f"警告：读取手指 {i} 位置失败，无法保存。")
                return
            current_pos[str(i)] = pos

        saved_positions[slot_name] = current_pos
        save_positions_to_file()
        print(f"当前位置已成功保存到槽位 '{slot_name}': {current_pos}")

    except ModbusException as e:
        print(f"错误：保存位置时发生 Modbus 错误: {e}")
    except Exception as e:
        print(f"错误：保存位置时发生意外错误: {e}")

def load_and_set_positions(slot_name):
    """从指定槽位加载位置并命令 OHand 移动。"""
    global hand_client, saved_positions
    if not hand_client or not hand_client.is_connected:
        print("错误：无法加载位置，未连接到 OHand。")
        return

    print(f"尝试从槽位 '{slot_name}' 加载并设置位置...")
    if slot_name in saved_positions:
        positions_to_set_dict = saved_positions[slot_name]
        positions_list = [0] * 6
        valid = True
        for i in range(6):
            key = str(i)
            if key in positions_to_set_dict:
                positions_list[i] = positions_to_set_dict[key]
            else:
                print(f"警告：槽位 '{slot_name}' 中缺少手指 {i} 的位置数据。")
                valid = False
                break

        if valid:
            print(f"正在命令 OHand 移动到位置: {positions_list}")
            try:
                if hand_client.set_all_fingers_target_pos(positions_list):
                    print("位置加载和设置命令已发送。")
                else:
                    print("错误：发送多位置设置命令失败。")
            except ModbusException as e:
                print(f"错误：加载位置时发生 Modbus 错误: {e}")
            except ValueError as e:
                 print(f"错误：加载的位置值无效: {e}")
            except Exception as e:
                print(f"错误：加载位置时发生意外错误: {e}")
        else:
             print(f"因数据不完整，未能设置槽位 '{slot_name}' 的位置。")

    else:
        print(f"错误：在已保存的位置中未找到槽位 '{slot_name}'。")

def move_finger(finger_index, direction):
    """根据方向移动指定手指一小步（逻辑位置）。"""
    global hand_client
    if not hand_client or not hand_client.is_connected:
        # print("错误：无法移动手指，未连接到 OHand。") # 减少打印
        return

    try:
        current_pos = hand_client.get_finger_current_pos(finger_index)
        if current_pos is None:
            print(f"错误：无法获取手指 {finger_index} 的当前位置。")
            return

        new_target_pos = current_pos + (direction * CONTROL_STEP)
        new_target_pos = max(MIN_POS, min(new_target_pos, MAX_POS))

        if not hand_client.set_finger_target_pos(finger_index, new_target_pos):
            print(f"错误：发送手指 {finger_index} 移动命令失败。")

    except ModbusException as e:
        # 减少频繁操作时的错误打印
        # print(f"错误：移动手指 {finger_index} 时发生 Modbus 错误: {e}")
        pass # 在快速按键时可能出现瞬时错误，暂时忽略
    except Exception as e:
        print(f"错误：移动手指 {finger_index} 时发生意外错误: {e}")


# --- 键盘监听回调 (与之前版本相同) ---

def on_press(key):
    """处理按键按下的事件，将命令放入队列。"""
    global listener_active, command_queue

    command = None
    try:
        # 尝试获取字符键，并转换为小写
        char = getattr(key, 'char', None)
        if char:
            char = char.lower()
            if char in control_key_map:
                command = ("move", control_key_map[char])
            elif char in load_key_map:
                command = ("load", load_key_map[char])

    except AttributeError:
       pass # 'char' 属性不存在，说明是特殊键

    # 如果不是字符键，检查是否是特殊键
    if command is None:
        if key == keyboard.Key.esc:
            command = ("exit", None)
        elif key in save_key_map:
            command = ("save", save_key_map[key])

    if command:
        command_queue.put(command)

    # 如果收到退出命令，也在这里停止监听器
    if command and command[0] == "exit":
        print("收到退出信号...")
        listener_active = False
        return False # 停止监听器

def keyboard_listener_thread():
    """运行键盘监听器的线程函数。"""
    global listener_active
    print("启动键盘监听器...")
    try:
        with keyboard.Listener(on_press=on_press) as listener:
            while listener_active:
                time.sleep(0.1)
            listener.stop()
    except Exception as e:
        print(f"键盘监听器线程出错: {e}")
        # 可能需要通知主线程退出
        if listener_active:
             command_queue.put(("exit", None)) # 尝试让主线程退出
    finally:
        print("键盘监听器已停止。")


# --- 主程序 ---
if __name__ == "__main__":
    print("--- OHand 键盘控制器 ---")
    print("控制按键:")
    print("  Q/A: 拇指弯曲 +/-  W/S: 食指 +/-    E/D: 中指 +/-")
    print("  R/F: 无名指 +/-  T/G: 小指 +/-    Y/H: 拇指旋转 +/-")
    print("功能按键:")
    # *** 更新帮助文本 ***
    print("  F1-F9: 保存当前所有手指位置")
    print("  1-9:   加载并移动到 F1-F9 保存的位置")
    print("  Esc:   退出程序")
    print("-" * 20)

    load_positions_from_file()

    hand_client = OHandModbusClient(port=SERIAL_PORT, slave_id=SLAVE_ID)

    try:
        if not hand_client.connect():
            print("无法连接到 OHand，程序退出。")
            exit(1)

        listener_thread = threading.Thread(target=keyboard_listener_thread, daemon=True)
        listener_thread.start()

        print("\n控制器已启动，按 Esc 退出。")

        while listener_active:
            try:
                cmd, value = command_queue.get(timeout=0.1)

                if cmd == "move":
                    finger_idx, direction = value
                    move_finger(finger_idx, direction)
                elif cmd == "save":
                    save_current_positions(value)
                elif cmd == "load":
                    load_and_set_positions(value)
                elif cmd == "exit":
                    print("正在退出主循环...")
                    listener_active = False
                    break

            except queue.Empty:
                continue
            except Exception as e:
                print(f"主循环处理命令时出错: {e}")
                # listener_active = False # Decide if error should stop the program
                # break

    except KeyboardInterrupt:
        print("\n检测到 Ctrl+C，正在退出...")
        listener_active = False
    except ModbusException as e:
         print(f"初始化或主循环中发生 Modbus 错误: {e}")
    except Exception as e:
        print(f"发生未处理的异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("正在进行清理...")
        listener_active = False

        if hand_client and hand_client.is_connected:
            hand_client.disconnect()

        print("程序已退出。")