#!/bin/bash
# 左贴边_两侧都堵。加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 左贴边_两侧都堵 "$@"
