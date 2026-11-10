#!/bin/bash
# 约 30 度斜向通道，右上到左下。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 斜线30度_右上到左下 "$@"
