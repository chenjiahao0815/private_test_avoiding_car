#!/bin/bash
# 两段反向 S 相接。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 双S_串联 "$@"
