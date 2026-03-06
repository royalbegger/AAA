#!/bin/bash
# 循环从0到299的所有序号，每个序号只运行一次
for n in {0..299} ; do
    # 执行测试脚本
    python3 run.py --world_idx $n
    # 每次运行后暂停5秒（如果不需要间隔，可删除这一行）
    sleep 8
done
