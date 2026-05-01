#!/bin/bash

# 提示输入
echo -n "Please enter world_idx: "
read WORLD_IDX

if [ -z "$WORLD_IDX" ]; then
    echo "world_idx is empty, exit."
    exit 1
fi

# # 激活 conda
# source ~/anaconda3/etc/profile.d/conda.sh
# conda activate RL_env

# source ROS workspace
source ../devel/setup.bash

# 启动程序
python3 run.py --world_idx ${WORLD_IDX}
