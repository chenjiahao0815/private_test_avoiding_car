#!/bin/bash
# 远侧等待时收不到 /car_information_all，3 秒和 4 秒暂停，等满 15 秒靠右。加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 矩形_动态车话题断流 "$@"
