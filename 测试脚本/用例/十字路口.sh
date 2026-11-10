#!/bin/bash
# 十字路口，沿横路穿过。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 十字路口 "$@"
