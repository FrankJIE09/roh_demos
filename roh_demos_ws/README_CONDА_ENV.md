# Conda环境与ROS集成使用说明

## 问题描述

`rosrun`默认使用系统的Python解释器，而不是conda环境中的Python。这会导致无法找到conda环境中安装的包（如mediapipe、opencv等）。

## 解决方案

### 方法1：使用环境设置脚本（推荐）

```bash
# 进入工作空间
cd ~/Frank/code/roh_demos/roh_demos_ws

# 设置环境并运行节点
./setup_env.sh rosrun Frank_control visual_hand_angle_service.py
```

### 方法2：使用conda环境运行脚本

```bash
# 使用专门的运行脚本
./run_with_conda.sh Frank_control visual_hand_angle_service.py
```

### 方法3：手动设置环境

```bash
# 激活conda环境
source /home/sage/anaconda3/etc/profile.d/conda.sh
conda activate roh_demos

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source ~/Frank/code/roh_demos/roh_demos_ws/devel/setup.bash

# 设置Python路径
export PYTHONPATH="/home/sage/anaconda3/envs/roh_demos/lib/python3.8/site-packages:$PYTHONPATH"
export PATH="/home/sage/anaconda3/envs/roh_demos/bin:$PATH"

# 运行节点
rosrun Frank_control visual_hand_angle_service.py
```

### 方法4：使用launch文件

在launch文件中指定Python解释器：

```xml
<launch>
    <node name="visual_hand_angle_service" pkg="Frank_control" type="visual_hand_angle_service.py" output="screen">
        <env name="PYTHONPATH" value="/home/sage/anaconda3/envs/roh_demos/lib/python3.8/site-packages:$(env PYTHONPATH)"/>
        <env name="PATH" value="/home/sage/anaconda3/envs/roh_demos/bin:$(env PATH)"/>
    </node>
</launch>
```

## 验证环境

运行以下命令验证环境是否正确设置：

```bash
./setup_env.sh python -c "import mediapipe; import cv2; print('环境设置成功!')"
```

## 常见问题

### 1. 找不到mediapipe模块
**解决方案：** 确保使用conda环境的Python解释器

### 2. 找不到cv2模块
**解决方案：** 在conda环境中安装opencv-python
```bash
conda activate roh_demos
pip install opencv-python
```

### 3. ROS节点启动失败
**解决方案：** 检查Python脚本的shebang行是否正确
```python
#!/home/sage/anaconda3/envs/roh_demos/bin/python3.8
```

## 自动化脚本

为了方便使用，可以创建别名：

```bash
# 添加到 ~/.bashrc
alias rosrun-conda='cd ~/Frank/code/roh_demos/roh_demos_ws && ./setup_env.sh rosrun'
```

然后使用：
```bash
rosrun-conda Frank_control visual_hand_angle_service.py
```

## 编译工作空间

使用conda环境编译：

```bash
cd ~/Frank/code/roh_demos/roh_demos_ws
./setup_env.sh catkin_make
```

## 测试

运行测试以验证环境：

```bash
./setup_env.sh python -c "
import rospy
import mediapipe as mp
import cv2
import numpy as np
print('所有依赖包导入成功!')
"
``` 