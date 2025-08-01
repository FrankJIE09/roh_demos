#!/bin/bash

# 设置conda环境用于ROS
export CONDA_ENV_NAME="roh_demos"
export CONDA_ENV_PATH="/home/sage/anaconda3/envs/roh_demos"

# 激活conda环境
source /home/sage/anaconda3/etc/profile.d/conda.sh
conda activate $CONDA_ENV_NAME

# 设置ROS环境变量以使用conda的Python
export ROS_PYTHON_VERSION=3
export PYTHONPATH=$CONDA_ENV_PATH/lib/python3.8/site-packages:$PYTHONPATH
export PATH=$CONDA_ENV_PATH/bin:$PATH

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source $(dirname "$0")/devel/setup.bash

echo "Conda环境已激活: $CONDA_ENV_NAME"
echo "Python路径: $(which python)"
echo "Python版本: $(python --version)"
echo "ROS环境已设置"

# 保持shell激活状态
exec "$@" 