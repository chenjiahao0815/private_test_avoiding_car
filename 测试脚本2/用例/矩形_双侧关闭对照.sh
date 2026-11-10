#!/bin/bash
# GARAGE_ENABLE_DUAL_SIDE_AVOIDANCE=false：老行为树，近侧堵住也绝不找远侧。加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 矩形_双侧关闭对照 "$@"
