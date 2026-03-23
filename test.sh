#!/bin/bash
# 循环从0到299的所有序号，每个序号只运行一次
for n in {0..299} ; do
    # 执行测试脚本
    python3 run.py --world_idx $n
done