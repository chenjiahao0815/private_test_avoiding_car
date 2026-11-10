#!/bin/bash
# 近侧没堵，直接停近侧。加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 矩形_近侧停靠 "$@"
