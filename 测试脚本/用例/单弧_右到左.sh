#!/bin/bash
# 单段圆弧（向上拱，半径 22 m），右到左。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 单弧_右到左 "$@"
