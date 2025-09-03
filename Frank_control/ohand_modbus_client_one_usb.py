#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
from pymodbus.client import ModbusSerialClient
from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder
from pymodbus.constants import Endian
from pymodbus.exceptions import ModbusException
import time
# 导入寄存器定义 (需要 roh_registers_v1.py 在同一目录或 Python 路径中)
try:
    from roh_registers_v1 import *
except ImportError:
    print("错误：未找到 roh_registers_v1.py 文件。请确保它在正确的目录中。")
    exit(1)

# --- 全局常量 ---
# 手指索引映射 (方便使用)
FINGER_THUMB_BEND = 0
FINGER_INDEX = 1
FINGER_MIDDLE = 2
FINGER_RING = 3
FINGER_PINKY = 4
FINGER_THUMB_ROT = 5
# 可以添加更多手指索引，如果寄存器列表中包含 6-9

# 状态码映射 (来自文档 4.3 节)
STATUS_MAP = {
    0: "STATUS_OPENING (正在展开)",
    1: "STATUS_CLOSING (正在抓取)",
    2: "STATUS_POS_REACHED (位置到位停止)",
    3: "STATUS_OVER_CURRENT (电流保护停止)",
    4: "STATUS_FORCE_REACHED (力控到位停止)",
    5: "STATUS_STUCK (电机堵转停止)",
}

# 错误子代码映射 (来自文档 3.4 节)
SUB_EXCEPTION_MAP = {
    1: "ERR_STATUS_INIT (等待初始化或正在初始化)",
    2: "ERR_STATUS_CALI (等待校正)",
    3: "ERR_INVALID_DATA (无效的寄存器值)",
    4: "ERR_STATUS_STUCK (电机堵转)",
    5: "ERR_OP_FAILED (操作失败)",
    6: "ERR_SAVE_FAILED (保存失败)",
}

# --- OHand Modbus 客户端类 ---

# In your ohand_modbus_client.py file

# ... (imports and global constants remain unchanged) ...

class OHandModbusClient:
    """
    用于通过 Modbus RTU 协议与多个 OHand 灵巧手交互的客户端类。
    使用 pymodbus 库进行串口通信，支持同时控制多个从站。
    """
    def __init__(self, port, slave_ids=None, baudrate=115200, parity='N', stopbits=1, bytesize=8, timeout=1):
        """
        初始化多手 Modbus 客户端。

        :param port: 串口端口号 (例如 '/dev/ttyUSB0' 或 'COM3').
        :param slave_ids: OHand 的 Modbus 从站 ID 列表 (例如 [4, 5, 6]) 或单个ID。
        :param baudrate: 波特率 (默认: 115200).
        :param parity: 校验位 ('N', 'E', 'O') (默认: 'N').
        :param stopbits: 停止位 (1, 1.5, 2) (默认: 1).
        :param bytesize: 数据位 (5, 6, 7, 8) (默认: 8).
        :param timeout: 通信超时时间 (秒) (默认: 1).
        """
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
            timeout=timeout
        )
        self.client.port = port
        self.port = port

        # 处理从站ID列表
        if slave_ids is None:
            self.slave_ids = [4, 5]  # 默认从站ID
        elif isinstance(slave_ids, (list, tuple)):
            self.slave_ids = list(slave_ids)
        else:
            self.slave_ids = [slave_ids]  # 单个ID转换为列表

        self.default_slave_id = self.slave_ids[0] if self.slave_ids else None
        self.is_connected = False

        # Modbus 标准通常使用大端字节序 (Big Endian)
        self.byte_order = Endian.BIG
        self.word_order = Endian.BIG

        # 配置日志记录器 (可选, 用于调试)
        logging.basicConfig()
        self.log = logging.getLogger(self.__class__.__name__)
        self.log.setLevel(logging.INFO) # 设置为 DEBUG 获取更详细信息

        print(f"初始化多手Modbus客户端:")
        print(f"  串口: {port}")
        print(f"  从站ID列表: {self.slave_ids}")
        print(f"  默认从站ID: {self.default_slave_id}")


    def connect(self, slave_id=None):
        """
        连接到 OHand 设备。

        :param slave_id: 可选的从站ID，用于测试特定从站连接。如果为None，则测试所有从站。
        :return: 连接成功的从站ID列表
        """
        print(f"尝试连接到 {self.client.port}...")

        if self.client.connect():
            self.is_connected = True
            print("串口连接成功.")

            # 测试从站连接
            if slave_id is not None:
                # 测试特定从站
                if self._test_slave_connection(slave_id):
                    print(f"从站 {slave_id} 连接成功.")
                    return [slave_id]
                else:
                    print(f"从站 {slave_id} 连接失败.")
                    return []
            else:
                # 测试所有从站
                connected_slaves = []
                for sid in self.slave_ids:
                    if self._test_slave_connection(sid):
                        print(f"从站 {sid} 连接成功.")
                        connected_slaves.append(sid)
                    else:
                        print(f"从站 {sid} 连接失败.")

                if connected_slaves:
                    print(f"成功连接 {len(connected_slaves)} 个从站: {connected_slaves}")
                else:
                    print("没有从站连接成功.")

                return connected_slaves
        else:
            self.is_connected = False
            print("串口连接失败.")
            return []

    def _test_slave_connection(self, slave_id):
        """测试特定从站的连接"""
        try:
            # 尝试读取节点ID来测试连接
            response = self.client.read_holding_registers(1005, 1, slave=slave_id)
            if response and not response.isError():
                return True
        except Exception:
            pass
        return False

    def disconnect(self):
        """断开与 OHand 设备的连接。"""
        if self.is_connected:
            # self.log.info("正在断开连接...")
            print("正在断开连接...")
            # Check if self.client.serial exists before closing, more robust
            if hasattr(self.client, 'serial') and self.client.serial.is_open:
                self.client.close()
            self.is_connected = False
            # self.log.info("连接已断开.")
            print("连接已断开.")

    # Modified: Added target_slave_id parameter
    def _read_registers(self, address, count=1, target_slave_id: int = None):
        """
        内部辅助函数：读取保持寄存器 (功能码 0x03)。

        :param address: 起始寄存器地址。
        :param count: 要读取的寄存器数量。
        :param target_slave_id: 可选的从站ID。如果提供，将用于此次读操作。
        :return: pymodbus 读取响应对象，如果失败则返回 None。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        """
        if not self.is_connected:
            print("读取寄存器错误：未连接。")
            raise ModbusException("客户端未连接")

        # Determine which slave ID to use
        slave_id_to_use = target_slave_id if target_slave_id is not None else self.default_slave_id

        try:
            # self.log.debug(f"读取寄存器: 地址={address}, 数量={count}, 从站ID={slave_id_to_use}")
            response = self.client.read_holding_registers(address, count, slave=slave_id_to_use)
            if response.isError():
                print(f"读取寄存器 {address} (ID: {slave_id_to_use}) 时发生 Modbus 错误: {response}")
                raise ModbusException(f"读取寄存器错误: {response}")
            # self.log.debug(f"读取成功: {response.registers}")
            return response
        except ModbusException as e:
            print(f"读取寄存器 {address} (ID: {slave_id_to_use}) 时发生 Modbus 异常: {e}")
            raise e
        except Exception as e:
            print(f"读取寄存器 {address} (ID: {slave_id_to_use}) 时发生意外错误: {e}")
            raise ModbusException(f"意外读取错误: {e}")


    # Modified: Added target_slave_id parameter
    def _write_single_register(self, address, value, target_slave_id: int = None):
        """
        内部辅助函数：写入单个保持寄存器 (功能码 0x06)。

        :param address: 要写入的寄存器地址。
        :param value: 要写入的 16 位值。
        :param target_slave_id: 可选的从站ID。如果提供，将用于此次写操作。
        :return: True 如果写入成功。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        """
        if not self.is_connected:
            print("写入寄存器错误：未连接。")
            raise ModbusException("客户端未连接")

        slave_id_to_use = target_slave_id if target_slave_id is not None else self.slave_id

        try:
            # self.log.debug(f"写入单个寄存器: 地址={address}, 值={value}, 从站ID={slave_id_to_use}")
            response = self.client.write_register(address, value, slave=slave_id_to_use)
            if response.isError():
                print(f"写入寄存器 {address} (ID: {slave_id_to_use}) 时发生 Modbus 错误: {response}")
                raise ModbusException(f"写入寄存器错误: {response}")
            # self.log.debug(f"写入成功。")
            return True
        except ModbusException as e:
            print(f"写入寄存器 {address} (ID: {slave_id_to_use}) 时发生 Modbus 异常: {e}")
            raise e
        except Exception as e:
            print(f"写入寄存器 {address} (ID: {slave_id_to_use}) 时发生意外错误: {e}")
            raise ModbusException(f"意外写入错误: {e}")

    # Modified: Added target_slave_id parameter
    def _write_multiple_registers(self, address, values, target_slave_id: int = None):
        """
        内部辅助函数：写入多个保持寄存器 (功能码 0x10)。

        :param address: 起始寄存器地址。
        :param values: 要写入的 16 位值列表。
        :param target_slave_id: 可选的从站ID。如果提供，将用于此次写操作。
        :return: True 如果写入成功。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        """
        if not self.is_connected:
            print("写入多个寄存器错误：未连接。")
            raise ModbusException("客户端未连接")

        slave_id_to_use = target_slave_id if target_slave_id is not None else self.slave_id

        try:
            # self.log.debug(f"写入多个寄存器: 地址={address}, 值={values}, 从站ID={slave_id_to_use}")
            response = self.client.write_registers(address, values, slave=slave_id_to_use)
            if response.isError():
                print(f"写入多个寄存器 {address} (ID: {slave_id_to_use}) 时发生 Modbus 错误: {response}")
                raise ModbusException(f"写入多个寄存器错误: {response}")
            # self.log.debug(f"写入成功。")
            return True
        except ModbusException as e:
            print(f"写入多个寄存器 {address} (ID: {slave_id_to_use}) 时发生 Modbus 异常: {e}")
            raise e
        except Exception as e:
            print(f"写入多个寄存器 {address} (ID: {slave_id_to_use}) 时发生意外错误: {e}")
            raise ModbusException(f"意外写入错误: {e}")

    # --- Now, modify the public methods that use these internal helpers ---

    # For Read-Only methods, add target_slave_id:
    def get_protocol_version(self, target_slave_id: int = None): # Added target_slave_id
        """读取协议版本号 (ROH_PROTOCOL_VERSION, Addr: 1000, R)"""
        response = self._read_registers(ROH_PROTOCOL_VERSION, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        if response:
            raw_value = response.registers[0]
            major = (raw_value >> 8) & 0xFF
            minor = raw_value & 0xFF
            return major, minor
        return None

    def get_firmware_version(self, target_slave_id: int = None): # Added target_slave_id
        """读取固件版本号 (ROH_FW_VERSION, Addr: 1001, R)"""
        response = self._read_registers(ROH_FW_VERSION, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        if response:
            raw_value = response.registers[0]
            major = (raw_value >> 8) & 0xFF
            minor = raw_value & 0xFF
            return major, minor
        return None

    def get_firmware_revision(self, target_slave_id: int = None): # Added target_slave_id
        """读取固件修订版本号 (ROH_FW_REVISION, Addr: 1002, R)"""
        response = self._read_registers(ROH_FW_REVISION, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        return response.registers[0] if response else None

    def get_hardware_version(self, target_slave_id: int = None): # Added target_slave_id
        """读取硬件版本号 (ROH_HW_VERSION, Addr: 1003, R)"""
        response = self._read_registers(ROH_HW_VERSION, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        if response:
            raw_value = response.registers[0]
            hw_type = (raw_value >> 8) & 0xFF
            hw_ver = raw_value & 0xFF
            return hw_type, hw_ver
        return None

    def get_bootloader_version(self, target_slave_id: int = None): # Added target_slave_id
        """读取 Bootloader 版本号 (ROH_BOOT_VERSION, Addr: 1004, R)"""
        response = self._read_registers(ROH_BOOT_VERSION, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        if response:
            raw_value = response.registers[0]
            major = (raw_value >> 8) & 0xFF
            minor = raw_value & 0xFF
            return major, minor
        return None

    # - 基本配置 (R/W 或 W) -
    def get_node_id(self, target_slave_id: int = None): # Added target_slave_id
        """读取灵巧手节点 ID (ROH_NODE_ID, Addr: 1005, R)"""
        response = self._read_registers(ROH_NODE_ID, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        return (response.registers[0] & 0xFF) if response else None

    def set_node_id(self, node_id, target_slave_id: int = None): # Added target_slave_id
        """设置灵巧手节点 ID (ROH_NODE_ID, Addr: 1005, W)。写入成功后 ROH 会保存并重启。"""
        if 0 <= node_id <= 255:
            return self._write_single_register(ROH_NODE_ID, node_id, target_slave_id=target_slave_id) # Passed target_slave_id
        else:
            print("错误: Node ID 必须在 0-255 之间。")
            return False

    def get_sub_exception_code(self, target_slave_id: int = None): # Added target_slave_id
        """读取错误子代码 (ROH_SUB_EXCEPTION, Addr: 1006, R)。用于获取 EC04 设备故障的具体原因。"""
        response = self._read_registers(ROH_SUB_EXCEPTION, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        if response:
            code = response.registers[0]
            description = SUB_EXCEPTION_MAP.get(code, f"未知子错误代码: {code}")
            return code, description
        return None, "读取失败"

    def get_battery_voltage(self, target_slave_id: int = None): # Added target_slave_id
        """读取电池电压值 (ROH_BATTERY_VOLTAGE, Addr: 1007, R)。单位 mV。(文档注记：暂时不可用)"""
        print("警告: 读取电池电压：根据文档，此功能暂时不可用。")
        response = self._read_registers(ROH_BATTERY_VOLTAGE, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        return response.registers[0] if response else None # 单位 mV

    def get_self_test_level(self, target_slave_id: int = None): # Added target_slave_id
        """读取开机自检开关设置 (ROH_SELF_TEST_LEVEL, Addr: 1008, R)"""
        response = self._read_registers(ROH_SELF_TEST_LEVEL, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        return response.registers[0] if response else None

    def set_self_test_level(self, level, target_slave_id: int = None): # Added target_slave_id
        """
        设置开机自检开关 (ROH_SELF_TEST_LEVEL, Addr: 1008, W)。设置时保存到非易失存储器。
        0: 等待 ROH_START_INIT 写 1 自检
        1: 允许开机归零 (默认)
        2: 允许开机完整自检
        """
        if level in [0, 1, 2]:
            return self._write_single_register(ROH_SELF_TEST_LEVEL, level, target_slave_id=target_slave_id) # Passed target_slave_id
        else:
            print(f"错误: 设置自检级别错误: 无效的级别 {level}。必须是 0, 1, 或 2。")
            return False

    def get_beep_switch(self, target_slave_id: int = None): # Added target_slave_id
        """读取蜂鸣器开关状态 (ROH_BEEP_SWITCH, Addr: 1009, R)。1: 允许发声, 0: 静音。"""
        response = self._read_registers(ROH_BEEP_SWITCH, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        return response.registers[0] if response else None

    def set_beep_switch(self, enable, target_slave_id: int = None): # Added target_slave_id
        """设置蜂鸣器开关状态 (ROH_BEEP_SWITCH, Addr: 1009, W)。设置时保存到非易失存储器。"""
        value = 1 if enable else 0
        return self._write_single_register(ROH_BEEP_SWITCH, value, target_slave_id=target_slave_id) # Passed target_slave_id

    def set_beep_period(self, duration_ms, target_slave_id: int = None): # Added target_slave_id
        """使蜂鸣器发声指定时间 (ROH_BEEP_PERIOD, Addr: 1010, W)。单位毫秒。"""
        if 0 <= duration_ms <= 65535:
             return self._write_single_register(ROH_BEEP_PERIOD, duration_ms, target_slave_id=target_slave_id) # Passed target_slave_id
        else:
            print(f"错误: 设置蜂鸣器周期错误: 时长 {duration_ms} 超出范围 (0-65535ms)。")
            return False

    def get_button_press_count(self, target_slave_id: int = None): # Added target_slave_id
        """读取按键按下次数 (ROH_BUTTON_PRESS_CNT, Addr: 1011, R)。主要用于校正时确认。"""
        response = self._read_registers(ROH_BUTTON_PRESS_CNT, 1, target_slave_id=target_slave_id) # Passed target_slave_id
        return response.registers[0] if response else None

    def set_button_press_count(self, count, target_slave_id: int = None): # Added target_slave_id
        """设置按键按下次数 (ROH_BUTTON_PRESS_CNT, Addr: 1011, W)。主要用于校正时确认。"""
        if 0 <= count <= 65535:
            return self._write_single_register(ROH_BUTTON_PRESS_CNT, count, target_slave_id=target_slave_id) # Passed target_slave_id
        else:
            print(f"错误: 设置按键次数错误: 次数 {count} 超出范围 (0-65535)。")
            return False

    # - 控制指令 (W) -
    def set_recalibrate(self, key_value, target_slave_id: int = None): # Added target_slave_id
        """
        请求重新校正 (ROH_RECALIBRATE, Addr: 1012, W)。
        需要写入特定值（非公开）让 ROH 灵巧手进入校正状态。
        """
        print("警告: 调用重新校正：需要特定的非公开值。")
        if 0 <= key_value <= 65535:
            return self._write_single_register(ROH_RECALIBRATE, key_value, target_slave_id=target_slave_id) # Passed target_slave_id
        else:
            print("错误: 重新校正错误：值超出范围 (0-65535)。")
            return False

    def start_initialization(self, target_slave_id: int = None): # Added target_slave_id
        """开始自检 (ROH_START_INIT, Addr: 1013, W)。仅当自检级别设为 0 时有效。"""
        return self._write_single_register(ROH_START_INIT, 1, target_slave_id=target_slave_id) # Passed target_slave_id

    def reset_device(self, dfu_mode=False, target_slave_id: int = None): # Added target_slave_id
        """
        复位设备 (ROH_RESET, Addr: 1014, W)。
        :param dfu_mode: 如果为 True，则写入 0 以外的值使设备重启进入 DFU 模式。
                         如果为 False (默认)，则写入 0 使设备重启到工作模式。
        :param target_slave_id: 可选的从站ID。
        """
        value = 1 if dfu_mode else 0
        return self._write_single_register(ROH_RESET, value, target_slave_id=target_slave_id) # Passed target_slave_id

    def power_off_device(self, target_slave_id: int = None): # Added target_slave_id
        """关机 (ROH_POWER_OFF, Addr: 1015, W)。(文档注记：暂时不可用)"""
        print("警告: 关机：根据文档，此功能暂时不可用。")
        return self._write_single_register(ROH_POWER_OFF, 1, target_slave_id=target_slave_id) # Passed target_slave_id

    # - 校准数据 (R/W) -
    def get_cali_end(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指运行区间上限（绝对位置）(ROH_CALI_END[0-5], Addr: 1020-1025, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_CALI_END0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取校准上限错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def get_cali_start(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指运行区间下限（绝对位置）(ROH_CALI_START[0-5], Addr: 1030-1035, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_CALI_START0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取校准下限错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def get_cali_thumb_preset_pos(self, preset_index, target_slave_id: int = None): # Added target_slave_id
        """读取大拇指旋转预设位置（绝对位置）(ROH_CALI_THUMB_POS[0-2], Addr: 1040-1042, R)。"""
        if 0 <= preset_index <= 2:
            address = ROH_CALI_THUMB_POS0 + preset_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取拇指预设位置错误：无效的预设索引 {preset_index} (应为 0-2)。")
            return None

    # - PID 参数 (R/W) -
    # Modified: Added target_slave_id parameter
    def _get_pid_param(self, base_address, finger_index, target_slave_id: int = None):
        """内部辅助函数：读取单个 PID 参数（乘以 100 存储）。"""
        if 0 <= finger_index <= 5:
            address = base_address + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return (response.registers[0] / 100.0) if response else None
        else:
            print(f"错误: 获取 PID 参数错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    # Modified: Added target_slave_id parameter
    def _set_pid_param(self, base_address, finger_index, value_float, target_slave_id: int = None):
        """内部辅助函数：写入单个 PID 参数（乘以 100 存储）。"""
        if 0 <= finger_index <= 5:
            value_int = int(round(value_float * 100.0))
            if 0 <= value_int <= 65535:
                address = base_address + finger_index
                return self._write_single_register(address, value_int, target_slave_id=target_slave_id) # Passed target_slave_id
            else:
                print(f"错误: 设置 PID 参数错误：转换后的值 {value_int} 超出范围 (0-65535)。原始值: {value_float}")
                return False
        else:
            print(f"错误: 设置 PID 参数错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_p(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的 P 值 (ROH_FINGER_P[0-5], Addr: 1045-1050, R)。"""
        return self._get_pid_param(ROH_FINGER_P0, finger_index, target_slave_id=target_slave_id) # Passed target_slave_id

    def set_finger_p(self, finger_index, value_float, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的 P 值 (ROH_FINGER_P[0-5], Addr: 1045-1050, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_P0, finger_index, value_float, target_slave_id=target_slave_id) # Passed target_slave_id

    def get_finger_i(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的 I 值 (ROH_FINGER_I[0-5], Addr: 1055-1060, R)。"""
        return self._get_pid_param(ROH_FINGER_I0, finger_index, target_slave_id=target_slave_id) # Passed target_slave_id

    def set_finger_i(self, finger_index, value_float, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的 I 值 (ROH_FINGER_I[0-5], Addr: 1055-1060, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_I0, finger_index, value_float, target_slave_id=target_slave_id) # Passed target_slave_id

    def get_finger_d(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的 D 值 (ROH_FINGER_D[0-5], Addr: 1065-1070, R)。"""
        return self._get_pid_param(ROH_FINGER_D0, finger_index, target_slave_id=target_slave_id) # Passed target_slave_id

    def set_finger_d(self, finger_index, value_float, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的 D 值 (ROH_FINGER_D[0-5], Addr: 1065-1070, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_D0, finger_index, value_float, target_slave_id=target_slave_id) # Passed target_slave_id

    def get_finger_g(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的 G 值 (抗重力?) (ROH_FINGER_G[0-5], Addr: 1075-1080, R)。"""
        return self._get_pid_param(ROH_FINGER_G0, finger_index, target_slave_id=target_slave_id) # Passed target_slave_id

    def set_finger_g(self, finger_index, value_float, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的 G 值 (ROH_FINGER_G[0-5], Addr: 1075-1080, W)。会保存到非易失存储。"""
        return self._set_pid_param(ROH_FINGER_G0, finger_index, value_float, target_slave_id=target_slave_id) # Passed target_slave_id

    # - 状态读取 (R) -
    def get_finger_status(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的状态码 (ROH_FINGER_STATUS[0-5], Addr: 1085-1090, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_STATUS0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            if response:
                code = response.registers[0]
                description = STATUS_MAP.get(code, f"未知状态码: {code}")
                return code, description
            return None, "读取失败"
        else:
            print(f"错误: 获取手指状态错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None, "无效索引"

    def get_finger_current_limit(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的电机电流限制值 (ROH_FINGER_CURRENT_LIMIT[0-5], Addr: 1095-1100, R)。单位 mA。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_CURRENT_LIMIT0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取电流限制错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def set_finger_current_limit(self, finger_index, limit_ma, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的电机电流限制值 (ROH_FINGER_CURRENT_LIMIT[0-5], Addr: 1095-1100, W)。单位 mA。开机时恢复为默认值。"""
        if 0 <= finger_index <= 5:
            if 0 <= limit_ma <= 65535:
                address = ROH_FINGER_CURRENT_LIMIT0 + finger_index
                return self._write_single_register(address, limit_ma, target_slave_id=target_slave_id) # Passed target_slave_id
            else:
                print(f"错误: 设置电流限制错误：值 {limit_ma} 超出范围 (0-65535 mA)。")
                return False
        else:
            print(f"错误: 设置电流限制错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_current(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的当前电机电流值 (ROH_FINGER_CURRENT[0-5], Addr: 1105-1110, R)。单位 mA。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_CURRENT0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取电流错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def get_finger_force_limit(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的力量限制值 (ROH_FINGER_FORCE_LIMIT[0-4], Addr: 1115-1119, R)。单位 mN。"""
        if 0 <= finger_index <= 4: # Note: thumb rotation (index 5) doesn't have force limit register
            address = ROH_FINGER_FORCE_LIMIT0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取力量限制错误：无效的手指索引 {finger_index} (应为 0-4)。")
            return None

    def set_finger_force_limit(self, finger_index, limit_mn, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的力量限制值 (ROH_FINGER_FORCE_LIMIT[0-4], Addr: 1115-1119, W)。单位 mN。开机时恢复为默认值。"""
        if 0 <= finger_index <= 4: # Note: thumb rotation (index 5) doesn't have force limit register
            if 0 <= limit_mn <= 65535:
                address = ROH_FINGER_FORCE_LIMIT0 + finger_index
                return self._write_single_register(address, limit_mn, target_slave_id=target_slave_id) # Passed target_slave_id
            else:
                print(f"错误: 设置力量限制错误：值 {limit_mn} 超出范围 (0-65535 mN)。")
                return False
        else:
            print(f"错误: 设置力量限制错误：无效的手指索引 {finger_index} (应为 0-4)。")
            return False

    def get_finger_force(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的当前力量值 (ROH_FINGER_FORCE[0-4], Addr: 1120-1124, R)。单位 mN。"""
        if 0 <= finger_index <= 4: # Note: thumb rotation (index 5) doesn't have force register
            address = ROH_FINGER_FORCE0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取力量错误：无效的手指索引 {finger_index} (应为 0-4)。")
            return None

    # - 运动控制 (R/W) -
    def get_finger_speed(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的逻辑速度 (ROH_FINGER_SPEED[0-5], Addr: 1125-1130, R)。单位: 逻辑位置/秒。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_SPEED0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取速度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def set_finger_speed(self, finger_index, speed, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的逻辑速度 (ROH_FINGER_SPEED[0-5], Addr: 1125-1130, W)。单位: 逻辑位置/秒。开机时恢复为默认值 (65535)。"""
        if 0 <= finger_index <= 5:
            if 0 <= speed <= 65535:
                address = ROH_FINGER_SPEED0 + finger_index
                return self._write_single_register(address, speed, target_slave_id=target_slave_id) # Passed target_slave_id
            else:
                print(f"错误: 设置速度错误：值 {speed} 超出范围 (0-65535)。")
                return False
        else:
            print(f"错误: 设置速度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_target_pos(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的逻辑目标位置 (ROH_FINGER_POS_TARGET[0-5], Addr: 1135-1140, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_POS_TARGET0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取目标位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    # Modified: Added target_slave_id parameter to the main control method
    def set_finger_target_pos(self, finger_index, position, target_slave_id: int = None): # <<<<<< THIS IS THE KEY CHANGE
        """设置指定手指的逻辑目标位置 (ROH_FINGER_POS_TARGET[0-5], Addr: 1135-1140, W)。写入后手指会开始移动。"""
        if 0 <= finger_index <= 5:
            if 0 <= position <= 65535:
                address = ROH_FINGER_POS_TARGET0 + finger_index
                return self._write_single_register(address, position, target_slave_id=target_slave_id) # Passed target_slave_id
            else:
                print(f"错误: 设置目标位置错误：值 {position} 超出范围 (0-65535)。")
                return False
        else:
            print(f"错误: 设置目标位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_current_pos(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的当前逻辑位置 (ROH_FINGER_POS[0-5], Addr: 1145-1150, R)。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_POS0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            return response.registers[0] if response else None
        else:
            print(f"错误: 获取当前位置错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def get_finger_target_angle(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的目标角度 (ROH_FINGER_ANGLE_TARGET[0-5], Addr: 1155-1160, R)。单位：度。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_ANGLE_TARGET0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            if response:
                decoder = BinaryPayloadDecoder.fromRegisters(response.registers, byteorder=self.byte_order, wordorder=self.word_order)
                scaled_value = decoder.decode_16bit_int()
                return scaled_value / 100.0
            return None
        else:
            print(f"错误: 获取目标角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    def set_finger_target_angle(self, finger_index, angle_deg, target_slave_id: int = None): # Added target_slave_id
        """设置指定手指的目标角度 (ROH_FINGER_ANGLE_TARGET[0-5], Addr: 1155-1160, W)。单位：度。写入后手指会开始移动。"""
        if 0 <= finger_index <= 5:
            scaled_value = int(round(angle_deg * 100.0))
            if -32768 <= scaled_value <= 32767:
                address = ROH_FINGER_ANGLE_TARGET0 + finger_index
                builder = BinaryPayloadBuilder(byteorder=self.byte_order, wordorder=self.word_order)
                builder.add_16bit_int(scaled_value)
                payload = builder.to_registers()
                return self._write_single_register(address, payload[0], target_slave_id=target_slave_id) # Passed target_slave_id
            else:
                print(f"错误: 设置目标角度错误：转换后的值 {scaled_value} 超出 int16 范围。原始角度: {angle_deg}")
                return False
        else:
            print(f"错误: 设置目标角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return False

    def get_finger_current_angle(self, finger_index, target_slave_id: int = None): # Added target_slave_id
        """读取指定手指的当前角度 (ROH_FINGER_ANGLE[0-5], Addr: 1165-1170, R)。单位：度。"""
        if 0 <= finger_index <= 5:
            address = ROH_FINGER_ANGLE0 + finger_index
            response = self._read_registers(address, 1, target_slave_id=target_slave_id) # Passed target_slave_id
            if response:
                decoder = BinaryPayloadDecoder.fromRegisters(response.registers, byteorder=self.byte_order, wordorder=self.word_order)
                scaled_value = decoder.decode_16bit_int()
                return scaled_value / 100.0
            return None
        else:
            print(f"错误: 获取当前角度错误：无效的手指索引 {finger_index} (应为 0-5)。")
            return None

    # --- Convenience methods (examples) ---
    def get_finger_status_all(self, target_slave_id: int = None): # Added target_slave_id
        """获取指定从站所有手指的状态。"""
        status_dict = {}
        for i in range(6): # 0 to 5
            try:
                code, desc = self.get_finger_status(i, target_slave_id=target_slave_id) # Passed target_slave_id
                status_dict[i] = {"code": code, "description": desc}
            except ModbusException:
                status_dict[i] = {"code": None, "description": "读取失败"}
        return status_dict

    def get_finger_positions(self, target_slave_id: int = None): # Added target_slave_id
        """获取指定从站所有手指的当前逻辑位置。"""
        pos_dict = {}
        for i in range(6):
            try:
                pos = self.get_finger_current_pos(i, target_slave_id=target_slave_id) # Passed target_slave_id
                pos_dict[i] = pos
            except ModbusException:
                pos_dict[i] = None
        return pos_dict

    def get_finger_angles(self, target_slave_id: int = None): # Added target_slave_id
        """获取指定从站所有手指的当前角度。"""
        angle_dict = {}
        for i in range(6):
            try:
                angle = self.get_finger_current_angle(i, target_slave_id=target_slave_id) # Passed target_slave_id
                angle_dict[i] = angle
            except ModbusException:
                angle_dict[i] = None
        return angle_dict

    def set_finger_positions(self, positions, target_slave_id: int = None): # Added target_slave_id
        """
        使用一次 Modbus 写多个寄存器 (0x10) 指令设置指定从站所有手指的目标逻辑位置。

        :param positions: 包含 6 个目标位置值的列表或元组 (对应手指 0-5)。
                          如果列表长度不足 6，则只设置前面的手指。
                          列表中的值应在 0-65535 之间。
        :param target_slave_id: 可选的从站ID。
        :return: True 如果写入成功。
        :raises ModbusException: 如果发生 Modbus 通信错误。
        :raises ValueError: 如果输入列表无效。
        """
        if not isinstance(positions, (list, tuple)):
            raise ValueError("输入必须是列表或元组")
        if not (0 < len(positions) <= 6):
             raise ValueError("列表长度必须在 1 到 6 之间")

        values_to_write = []
        for pos in positions:
            if not (0 <= pos <= 65535):
                raise ValueError(f"位置值 {pos} 超出范围 (0-65535)")
            values_to_write.append(pos)

        if not values_to_write:
            return True # Nothing to write

        start_address = ROH_FINGER_POS_TARGET0
        return self._write_multiple_registers(start_address, values_to_write, target_slave_id=target_slave_id) # Passed target_slave_id

    # --- 多手控制方法 ---

    def get_all_hands_info(self):
        """获取所有从站的基本信息"""
        hands_info = {}
        for slave_id in self.slave_ids:
            try:
                info = {
                    'protocol_version': self.get_protocol_version(target_slave_id=slave_id),
                    'firmware_version': self.get_firmware_version(target_slave_id=slave_id),
                    'node_id': self.get_node_id(target_slave_id=slave_id),
                    'hardware_version': self.get_hardware_version(target_slave_id=slave_id)
                }
                hands_info[slave_id] = info
            except Exception as e:
                print(f"获取从站{slave_id}信息失败: {e}")
                hands_info[slave_id] = None
        return hands_info

    def get_all_hands_status(self):
        """获取所有从站的手指状态"""
        hands_status = {}
        for slave_id in self.slave_ids:
            try:
                status_dict = self.get_finger_status_all(target_slave_id=slave_id)
                hands_status[slave_id] = status_dict
            except Exception as e:
                print(f"获取从站{slave_id}状态失败: {e}")
                hands_status[slave_id] = None
        return hands_status

    def get_all_hands_positions(self):
        """获取所有从站的手指当前位置"""
        hands_positions = {}
        for slave_id in self.slave_ids:
            try:
                positions = self.get_finger_positions(target_slave_id=slave_id)
                hands_positions[slave_id] = positions
            except Exception as e:
                print(f"获取从站{slave_id}位置失败: {e}")
                hands_positions[slave_id] = None
        return hands_positions

    def get_all_hands_angles(self):
        """获取所有从站的手指当前角度"""
        hands_angles = {}
        for slave_id in self.slave_ids:
            try:
                angles = self.get_finger_angles(target_slave_id=slave_id)
                hands_angles[slave_id] = angles
            except Exception as e:
                print(f"获取从站{slave_id}角度失败: {e}")
                hands_angles[slave_id] = None
        return hands_angles

    def set_all_hands_positions(self, positions_dict):
        """
        同时设置多个从站的手指位置

        :param positions_dict: 字典，键为从站ID，值为位置列表
        :return: 成功设置的从站ID列表
        """
        success_slaves = []
        for slave_id, positions in positions_dict.items():
            try:
                if self.set_finger_positions(positions, target_slave_id=slave_id):
                    success_slaves.append(slave_id)
                else:
                    print(f"设置从站{slave_id}位置失败")
            except Exception as e:
                print(f"设置从站{slave_id}位置异常: {e}")
        return success_slaves

    def set_all_hands_angles(self, angles_dict):
        """
        同时设置多个从站的手指角度

        :param angles_dict: 字典，键为从站ID，值为角度列表
        :return: 成功设置的从站ID列表
        """
        success_slaves = []
        for slave_id, angles in angles_dict.items():
            try:
                # 逐个设置每个手指的角度
                success = True
                for finger_idx, angle in enumerate(angles):
                    if not self.set_finger_target_angle(finger_idx, angle, target_slave_id=slave_id):
                        success = False
                        break
                if success:
                    success_slaves.append(slave_id)
                else:
                    print(f"设置从站{slave_id}角度失败")
            except Exception as e:
                print(f"设置从站{slave_id}角度异常: {e}")
        return success_slaves

    def set_all_hands_speed(self, speed_dict):
        """
        同时设置多个从站的手指速度

        :param speed_dict: 字典，键为从站ID，值为速度列表
        :return: 成功设置的从站ID列表
        """
        success_slaves = []
        for slave_id, speeds in speed_dict.items():
            try:
                success = True
                for finger_idx, speed in enumerate(speeds):
                    if not self.set_finger_speed(finger_idx, speed, target_slave_id=slave_id):
                        success = False
                        break
                if success:
                    success_slaves.append(slave_id)
                else:
                    print(f"设置从站{slave_id}速度失败")
            except Exception as e:
                print(f"设置从站{slave_id}速度异常: {e}")
        return success_slaves

    def execute_synchronized_action(self, action_func, *args, **kwargs):
        """
        在所有从站上执行同步动作

        :param action_func: 要执行的函数名（字符串）
        :param args: 函数参数
        :param kwargs: 函数关键字参数
        :return: 成功执行的从站ID列表
        """
        success_slaves = []
        for slave_id in self.slave_ids:
            try:
                # 动态调用方法
                if hasattr(self, action_func):
                    method = getattr(self, action_func)
                    # 添加target_slave_id参数
                    kwargs['target_slave_id'] = slave_id
                    if method(*args, **kwargs):
                        success_slaves.append(slave_id)
                    else:
                        print(f"从站{slave_id}执行{action_func}失败")
                else:
                    print(f"方法{action_func}不存在")
            except Exception as e:
                print(f"从站{slave_id}执行{action_func}异常: {e}")
        return success_slaves




# --- Main program example (keep as is, or remove if this is purely a library file) ---
# ... (rest of the file for example usage remains unchanged) ...

# --- 主程序示例 ---
if __name__ == "__main__":
    # --- 配置 ---
    SERIAL_PORT = '/dev/ttyUSB0'  # 在 Linux 上可能是这个，Windows 上可能是 'COMx'
    # SERIAL_PORT = 'COM3'       # Windows 示例
    SLAVE_IDS = [4, 5]          # 多个 OHand 的 Modbus 从站地址

    # --- 创建多手客户端实例 ---
    multi_hand = OHandModbusClient(port=SERIAL_PORT, slave_ids=SLAVE_IDS)

    try:
        # --- 连接所有从站 ---
        connected_slaves = multi_hand.connect()
        if not connected_slaves:
            print("没有从站连接成功，程序退出")
            exit(1)

        print(f"成功连接的从站: {connected_slaves}")

        # --- 读取所有从站信息 ---
        print("\n--- 读取所有从站信息 ---")
        all_hands_info = multi_hand.get_all_hands_info()
        for slave_id, info in all_hands_info.items():
            if info:
                print(f"从站{slave_id}:")
                print(f"  协议版本: {info['protocol_version']}")
                print(f"  固件版本: {info['firmware_version']}")
                print(f"  节点ID: {info['node_id']}")
                print(f"  硬件版本: {info['hardware_version']}")
            else:
                print(f"从站{slave_id}: 信息读取失败")

        # --- 读取所有从站状态 ---
        print("\n--- 读取所有从站状态 ---")
        all_hands_status = multi_hand.get_all_hands_status()
        for slave_id, status in all_hands_status.items():
            if status:
                print(f"从站{slave_id}手指状态:")
                for finger_idx, finger_status in status.items():
                    print(f"  手指{finger_idx}: {finger_status['description']}")
            else:
                print(f"从站{slave_id}: 状态读取失败")

        # --- 读取所有从站位置和角度 ---
        print("\n--- 读取所有从站位置和角度 ---")
        all_hands_positions = multi_hand.get_all_hands_positions()
        all_hands_angles = multi_hand.get_all_hands_angles()

        for slave_id in connected_slaves:
            positions = all_hands_positions.get(slave_id, {})
            angles = all_hands_angles.get(slave_id, {})
            print(f"从站{slave_id}:")
            print(f"  位置: {positions}")
            print(f"  角度: {angles}")

        # --- 多手控制示例 ---
        print("\n--- 多手控制示例 ---")

        # 设置所有从站的速度
        speed_dict = {}
        for slave_id in connected_slaves:
            speed_dict[slave_id] = [65535, 65535, 65535, 65535, 65535, 65535]  # 6个手指的速度

        success_slaves = multi_hand.set_all_hands_speed(speed_dict)
        print(f"速度设置成功的从站: {success_slaves}")

        # 示例1: 所有从站握拳
        print("\n1. 所有从站握拳")
        positions_dict = {}
        for slave_id in connected_slaves:
            positions_dict[slave_id] = [65535, 65535, 65535, 65535, 65535, 65535]  # 握拳位置

        success_slaves = multi_hand.set_all_hands_positions(positions_dict)
        print(f"握拳成功的从站: {success_slaves}")
        time.sleep(3)

        # 示例2: 所有从站张开
        print("\n2. 所有从站张开")
        positions_dict = {}
        for slave_id in connected_slaves:
            positions_dict[slave_id] = [0, 0, 0, 0, 0, 0]  # 张开位置

        success_slaves = multi_hand.set_all_hands_positions(positions_dict)
        print(f"张开成功的从站: {success_slaves}")
        time.sleep(3)

        # 示例3: 不同从站做不同动作
        print("\n3. 不同从站做不同动作")
        positions_dict = {}
        for i, slave_id in enumerate(connected_slaves):
            if i == 0:  # 第一个从站握拳
                positions_dict[slave_id] = [65535, 65535, 65535, 65535, 65535, 65535]
            else:  # 其他从站张开
                positions_dict[slave_id] = [0, 0, 0, 0, 0, 0]

        success_slaves = multi_hand.set_all_hands_positions(positions_dict)
        print(f"不同动作成功的从站: {success_slaves}")
        time.sleep(3)

        # 示例4: 使用直接的手指控制函数
        print("\n4. 使用直接的手指控制函数")
        for slave_id in connected_slaves:
            # 读取位置
            positions_dict = multi_hand.get_finger_positions(target_slave_id=slave_id)
            positions = [positions_dict.get(i, 0) for i in range(6)]
            print(f"从站{slave_id}当前位置: {positions}")
            
            # 设置位置
            new_positions = [32768, 32768, 0, 0, 0, 0]  # 拇指和食指弯曲
            success = multi_hand.set_finger_positions(new_positions, target_slave_id=slave_id)
            if success:
                print(f"从站{slave_id}新位置设置成功")
            time.sleep(2)

        # 恢复初始状态
        print("\n5. 恢复初始状态")
        for slave_id in connected_slaves:
            multi_hand.set_finger_positions([0, 0, 0, 0, 0, 0], target_slave_id=slave_id)
        print("所有从站已恢复初始状态")

    except ModbusException as e:
        print(f"发生 Modbus 错误: {e}")
    except KeyboardInterrupt:
        print("\n用户中断。")
    except Exception as e:
        print(f"发生未知错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # --- 断开连接 ---
        multi_hand.disconnect()