#!/bin/bash
# L 形圆角弯（半径 6 m），上进左出。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" L形圆弯_上进左出 "$@"
