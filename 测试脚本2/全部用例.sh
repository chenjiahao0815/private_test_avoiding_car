#!/bin/bash
# 依次跑全部双侧避车用例，每次一个日志序号，结果由 运行用例.sh 追加到 logs/汇总.md。
# 用法：./全部用例.sh [用例名关键字...]
#   ./全部用例.sh            全部已登记用例
#   ./全部用例.sh 远侧        只跑名字里带 远侧 的
# 每个用例都会重启整套仿真，单次约 3~5 分钟。中途 Ctrl+C 会停掉当前用例并退出。
ROOT="$(cd "$(dirname "$0")" && pwd)"
FILTERS=("$@")

mapfile -t ALL < <(python3 "$ROOT/容器/场景用例.py" --list | cut -f1)
SELECTED=()
for name in "${ALL[@]}"; do
  if [ ${#FILTERS[@]} -eq 0 ]; then SELECTED+=("$name"); continue; fi
  for f in "${FILTERS[@]}"; do [[ "$name" == *"$f"* ]] && { SELECTED+=("$name"); break; }; done
done
[ ${#SELECTED[@]} -gt 0 ] || { echo "没有匹配的用例"; exit 2; }

stopped=0
trap 'stopped=1' INT
pass=0
fail=0
for name in "${SELECTED[@]}"; do
  [ "$stopped" = 1 ] && break
  if "$ROOT/运行用例.sh" "$name" --no-hold < /dev/null; then
    pass=$((pass + 1))
  else
    fail=$((fail + 1))
  fi
done
echo "完成：PASS $pass，FAIL $fail。汇总：$ROOT/logs/汇总.md"

[ "$stopped" = 0 ] || exit 130
[ "$fail" = 0 ]
