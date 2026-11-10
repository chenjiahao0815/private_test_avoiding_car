#!/bin/bash
# 上下支路错开的双路口。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 错位路口 "$@"
