#!/bin/bash

# Frank Control 快速启动脚本

echo "=== Frank Control 快速启动 ==="

# 检查工作空间是否已编译
if [ ! -d "devel" ]; then
    echo "工作空间未编译，正在编译..."
    ./setup_env.sh catkin_make
fi

# 检查是否提供了参数
if [ $# -eq 0 ]; then
    echo "启动手部角度服务..."
    ./setup_env.sh rosrun Frank_control visual_hand_angle_service.py
else
    echo "运行指定命令: $@"
    ./setup_env.sh "$@"
fi 