#!/bin/bash
# 驶入后顺时针绕 270 度再向北驶出。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" 环线钩 "$@"
