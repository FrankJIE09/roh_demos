#!/usr/bin/env bash

# ====== 配置区（如有不同请自行修改）======
CONDA_ENV=roh_demos
WS_PATH=~/Frank/code/roh_demos/roh_demos_ws
PKG_NAME=Frank_control
NODE_SCRIPT=visual_hand_angle_service.py
# ==========================================

SCRIPT_PATH="$WS_PATH/src/$PKG_NAME/src/$NODE_SCRIPT"

echo "激活conda环境: $CONDA_ENV"
source ~/anaconda3/etc/profile.d/conda.sh
conda activate $CONDA_ENV

echo "source ROS工作空间: $WS_PATH/devel/setup.bash"
source $WS_PATH/devel/setup.bash

echo "直接用python运行节点: $SCRIPT_PATH"
python "$SCRIPT_PATH" 