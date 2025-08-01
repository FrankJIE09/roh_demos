#!/bin/bash

# 使用conda环境运行ROS节点的脚本

# 检查参数
if [ $# -eq 0 ]; then
    echo "用法: $0 <包名> <节点名> [参数...]"
    echo "示例: $0 Frank_control visual_hand_angle_service.py"
    exit 1
fi

PACKAGE_NAME=$1
NODE_NAME=$2
shift 2

# 设置conda环境
export CONDA_ENV_PATH="/home/sage/anaconda3/envs/roh_demos"
source /home/sage/anaconda3/etc/profile.d/conda.sh
conda activate roh_demos

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source $(dirname "$0")/devel/setup.bash

# 设置Python路径
export PYTHONPATH=$CONDA_ENV_PATH/lib/python3.8/site-packages:$PYTHONPATH
export PATH=$CONDA_ENV_PATH/bin:$PATH

echo "使用conda环境运行ROS节点..."
echo "包名: $PACKAGE_NAME"
echo "节点名: $NODE_NAME"
echo "Python路径: $(which python)"
echo "Python版本: $(python --version)"

# 运行ROS节点
rosrun $PACKAGE_NAME $NODE_NAME "$@" 