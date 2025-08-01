#!/bin/bash

# 设置ROS和conda环境的脚本

# 激活conda环境
source /home/sage/anaconda3/etc/profile.d/conda.sh
conda activate roh_demos

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source $(dirname "$0")/devel/setup.bash

# 设置Python环境变量
export PYTHONPATH="/home/sage/anaconda3/envs/roh_demos/lib/python3.8/site-packages:$PYTHONPATH"
export PATH="/home/sage/anaconda3/envs/roh_demos/bin:$PATH"

# 设置ROS Python版本
export ROS_PYTHON_VERSION=3

echo "环境设置完成:"
echo "Conda环境: roh_demos"
echo "Python路径: $(which python)"
echo "Python版本: $(python --version)"
echo "ROS环境: $(echo $ROS_DISTRO)"

# 如果提供了命令参数，则执行
if [ $# -gt 0 ]; then
    exec "$@"
fi 