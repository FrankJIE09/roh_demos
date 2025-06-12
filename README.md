# OHand 双臂灵巧手演示与控制套件

## 项目简介
本仓库提供了一套完整的 Python 脚本，演示如何通过 **键盘**、**摄像头视觉** 以及 **Modbus‑RTU** 协议实时控制两只 RobotEra OHand 灵巧手，实现握拳、张开、拇指旋转等多自由度动作。脚本同时支持 **单手** 与 **双手** 场景，并内置可视化调试工具与手部角度自动标定流程，方便快速上手与二次开发。

> **适用人群**：机器人研究人员、人机交互/义肢开发者、教学与展示、Demo 原型验证。

---

## 目录结构
| 文件 | 作用 | 典型使用场景 |
| ---- | ---- | ------------ |
| `ohand_modbus_client.py` | OHand **Modbus‑RTU** 通讯封装库 | 所有脚本的底层接口 |
| `keyboard_control.py` | **单手** 键盘步进控制，带 F1‑F9 姿态保存/加载 | 单手 |
| `dual_ohand_keyboard_control.py` | **双手** 键盘步进控制，支持独立控制左右手并保存 9 组姿态 | 双手 |
| `calibrate_hand_angles.py` | 使用 **Orbbec 相机 + MediaPipe** 采集极限姿态，生成 `hand_angle_calibration.json` | 标定 |
| `orbbec_ohand_controller.py` | 单手、基于 **距离阈值** 的视觉控制 | 单手实时跟随 |
| `dual_ohand_visual_control.py` | 双手、基于 **距离阈值** 的视觉控制 | 双手实时跟随 |
| `dual_ohand_visual_control_angle.py` | 双手、基于 **角度映射** 的视觉控制（需先标定） | 双手高精度控制 |

> 依赖的 `camera/` 目录需包含 Orbbec SDK 示例 `orbbec_camera.py`。

---

## 硬件依赖
1. **RobotEra OHand 灵巧手 ×2**（支持 Modbus‑RTU）。  
2. **USB‑RS485 转接器**：速率 ≥115 200 bps。  
3. **Orbbec 深度/彩色摄像头**（Prime、Femto 等型号实测可用）。  
4. Linux/Windows PC（建议 Ubuntu 20.04+/Windows 10，Python 3.9+）。

---

## 软件依赖
```bash
pip install -r requirements.txt
# requirements.txt 示范内容（根据实际环境增删）
mediapipe==0.10.*
opencv-python
numpy
pymodbus==3.*
pynput
```
- Orbbec SDK 驱动与 udev 规则（Linux）
- 摄像头固件/驱动 & USB3.0 线

---

## 快速开始
### 1️⃣ 连接硬件
- 将左右 OHand 分别连接至 `/dev/ttyUSB0` 与 `/dev/ttyUSB1`（或 Windows `COMx`）。
- 确保左右手 **Modbus Slave ID 不同**（默认示例均为 2，可在 `ohand_modbus_client.py` 中修改）。
- 接好 Orbbec 摄像头。

### 2️⃣ 创建并激活虚拟环境（可选）
```bash
python3 -m venv venv
source venv/bin/activate   # Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3️⃣ 手部角度标定（可选，但推荐）
```bash
python calibrate_hand_angles.py
```
根据窗口提示，依次保持手掌 **张开(o)**、**握拳(c)**、**拇指内收(i)**、**拇指外展(t)**，按 **s** 保存，按 **q** 退出并生成 `hand_angle_calibration.json`。

### 4️⃣ 视觉控制示例（双手角度映射）
```bash
python dual_ohand_visual_control_angle.py
```
> 若未执行步骤 3，将退回默认角度范围，请在脚本顶部 `DEFAULT_ANGLE_RANGES` 中手动调整。

### 5️⃣ 键盘步进控制示例（双手）
```bash
python dual_ohand_keyboard_control.py
```
- **左手**：`Q/A` 拇指弯曲 ±，`W/S` 食指 ±，`E/D` 中指 ±，`R/F` 无名指 ±，`T/G` 小指 ±，`Y/H` 拇指旋转 ±  
- **右手**：`U/J`、`I/K`、`O/L`、`P/;`、`[/'`、`]\` 同理（可在脚本 `left_control_map/right_control_map` 调整）  
- **保存姿态**：`F1‑F9`  保存当前左右手 6×2 DoF。  
- **加载姿态**：数字键 `1‑9` 调用相应槽位。  
- **退出**：`Esc`。

单手键盘控制请运行：
```bash
python keyboard_control.py
```

---

## 配置说明
所有与硬件相关的串口、Slave ID、波特率、角度/距离量程等参数均在各脚本顶部 **Constants and Configuration** 区域集中定义，直接修改即可；无需改动核心逻辑。

---

## 常见问题 FAQ
| 现象 | 可能原因 | 解决办法 |
| ---- | -------- | -------- |
| 运行脚本提示 `ImportError: roh_registers_v1.py not found` | 缺少寄存器定义文件 | 将官方 `roh_registers_v1.py` 放入同目录或 `PYTHONPATH` |
| 摄像头窗口黑屏 / `No Orbbec devices found` | 驱动未安装或权限不足 | 安装 Orbbec SDK，Linux 下 `sudo udevadm control --reload-rules` 后重新插拔 |
| OHand 未响应 / `connect failed` | 串口占用、线序错误、ID 冲突 | 检查线缆、电源、串口号、波特率，确保左右手 ID 唯一 |
| 姿态映射不准确 | 未校准 / 角度范围不匹配 | 重新运行 `calibrate_hand_angles.py` 或调整 `DEFAULT_ANGLE_RANGES` |

# ROH灵巧手 ModBus RTU Slave ID 扫描程序

这个程序用于扫描指定串口上ROH灵巧手设备的ModBus RTU slave ID。

## 功能特点

- 自动扫描指定串口上的所有可能的slave ID
- 显示找到的设备信息（协议版本、固件版本、硬件版本）
- 支持自定义扫描范围
- 支持快速扫描模式
- 详细的扫描进度显示

## 安装依赖

```bash
pip install -r requirements.txt
```

或者手动安装：

```bash
pip install pymodbus==2.5.3 pyserial==3.5
```

## 使用方法

### 基本用法

```bash
# Linux系统
python scan_modbus_slave_id.py /dev/ttyUSB0

# Windows系统  
python scan_modbus_slave_id.py COM3
```

### 高级用法

```bash
# 指定扫描范围
python scan_modbus_slave_id.py /dev/ttyUSB0 --start 1 --end 10

# 快速扫描模式（只扫描1-10）
python scan_modbus_slave_id.py /dev/ttyUSB0 --quick

# 调整超时时间
python scan_modbus_slave_id.py /dev/ttyUSB0 --timeout 0.2
```

### 参数说明

- `port`: 串口名称（必需参数）
  - Linux: `/dev/ttyUSB0`, `/dev/ttyACM0` 等
  - Windows: `COM1`, `COM3` 等
- `--start`: 起始slave ID（默认：1）
- `--end`: 结束slave ID（默认：247）
- `--timeout`: 查询超时时间，秒（默认：0.1）
- `--quick`: 快速扫描模式，只扫描1-10

## 示例输出

```
正在扫描串口 /dev/ttyUSB0 上的ModBus RTU设备...
扫描范围: slave ID 1 到 247
通信参数: 115200bps, 8N1
--------------------------------------------------
串口连接成功，开始扫描...
✓ 找到设备! Slave ID: 2, ROH_NODE_ID: 2
扫描进度: 10/247
扫描进度: 20/247
...

============================================================
扫描结果汇总:
============================================================
共找到 1 个设备:

设备 1:
  Slave ID: 2
  ROH Node ID: 2
  协议版本: 1.0
  固件版本: 2.1
  硬件版本: 1.0
```

## 技术说明

### 通信参数

根据ROH灵巧手ModBus RTU协议文档：
- 波特率：115200 bps
- 数据位：8
- 停止位：1
- 奇偶校验：无

### 检测原理

程序通过尝试读取ROH_NODE_ID寄存器（地址1005）来检测有效的slave ID：
- 如果读取成功，说明该slave ID上有ROH设备
- 同时获取设备的版本信息等详细参数

### 注意事项

1. 确保ROH设备已正确连接到串口
2. 确保串口没有被其他程序占用
3. 根据实际情况调整超时时间
4. 如果设备较多，建议使用指定范围扫描以提高效率

## 故障排除

### 常见问题

1. **串口连接失败**
   - 检查串口名称是否正确
   - 检查串口权限（Linux下可能需要sudo或添加用户到dialout组）
   - 确保串口没有被其他程序占用

2. **找不到设备**
   - 检查设备是否正确连接
   - 检查设备是否上电
   - 尝试调整超时时间（--timeout 0.5）
   - 检查通信线路是否正常

3. **扫描速度慢**
   - 使用快速扫描模式（--quick）
   - 指定较小的扫描范围（--start 1 --end 10）
   - 减小超时时间（但不要设置太小）

### Linux权限问题

如果遇到权限问题，可以：

```bash
# 方法1：添加用户到dialout组
sudo usermod -a -G dialout $USER
# 然后重新登录

# 方法2：临时使用sudo运行
sudo python scan_modbus_slave_id.py /dev/ttyUSB0
```


