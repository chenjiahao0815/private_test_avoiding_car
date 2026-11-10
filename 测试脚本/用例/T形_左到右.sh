#!/bin/bash
# T 形路口（支路朝下），沿横路行驶，左到右。加 --external 测外部停靠点，加 --no-hold 跑完直接退出。
exec "$(dirname "$0")/../运行用例.sh" T形_左到右 "$@"
