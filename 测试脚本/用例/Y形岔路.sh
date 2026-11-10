#!/bin/bash
# Y 形岔路，驶入左上支。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" Y形岔路 "$@"
