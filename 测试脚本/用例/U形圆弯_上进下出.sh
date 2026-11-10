#!/bin/bash
# U 形掉头弯（开口朝右，半径 6 m），上进下出。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" U形圆弯_上进下出 "$@"
