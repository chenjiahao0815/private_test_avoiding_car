#!/bin/bash
# 跑一个双侧避车用例：分配日志序号、同步脚本到容器、在容器里执行、把日志拷回 logs/序号_用例名/，结果追加到 logs/汇总.md。
# 用法：./运行用例.sh <用例名> [--no-hold]
#   --no-hold   跑完直接清理退出（默认保留画面，Ctrl+C 结束）
# 一般不直接调用，用 用例/ 下的脚本。
set -o pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
CTR=capella-ros-core-1
CASES="$ROOT/容器/场景用例.py"

CASE="${1:-}"
[ $# -gt 0 ] && shift
if [ -z "$CASE" ] || ! python3 "$CASES" --list | cut -f1 | grep -qx "$CASE"; then
  [ -n "$CASE" ] && echo "没有用例：$CASE"
  echo "用法：$0 <用例名> [--no-hold]，可选用例："
  python3 "$CASES" --list
  exit 2
fi
EXTRA=()
for arg in "$@"; do
  case "$arg" in
    --no-hold) EXTRA+=(--no-hold) ;;
    *) echo "未知参数：$arg"; exit 2 ;;
  esac
done

LOGS="$ROOT/logs"
mkdir -p "$LOGS"
exec 9> "$LOGS/.序号锁"
flock 9
last=$(ls -1 "$LOGS" | sed -n 's/^\([0-9]\{3,\}\)_.*/\1/p' | sort -n | tail -1)
ID="$(printf '%03d' $((10#${last:-0} + 1)))_${CASE}"
mkdir -p "$LOGS/$ID"
flock -u 9
exec 9>&-

"$ROOT/同步到容器.sh" > /dev/null || { echo "同步到容器失败"; exit 1; }
TTY=(-i)
[ -t 0 ] && [ -t 1 ] && TTY=(-it)
echo "[$ID] 开始"
# Ctrl+C 交给容器里的脚本清理，这里继续往下拷日志。
trap ':' INT
docker exec "${TTY[@]}" "$CTR" bash /capella/避车测试2/一键避车测试.sh --case "$CASE" --run-id "$ID" "${EXTRA[@]}"
trap - INT
docker cp "$CTR:/capella/避车测试2/logs/$ID/." "$LOGS/$ID/" > /dev/null
result=$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1])).get("result", "无结果"))' \
  "$LOGS/$ID/上位机/report.json" 2> /dev/null || echo "无结果（看 $LOGS/$ID 各程序日志）")
echo "[$ID] $result"
echo "日志：$LOGS/$ID"
SUMMARY="$LOGS/汇总.md"
[ -f "$SUMMARY" ] || printf '| 时间 | 日志 | 用例 | 结果 |\n| --- | --- | --- | --- |\n' > "$SUMMARY"
printf '| %s | %s | %s | %s |\n' "$(date '+%m-%d %H:%M')" "$ID" "$CASE" "${result//|//}" >> "$SUMMARY"
[[ "$result" == PASS* ]]
