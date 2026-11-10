#!/bin/bash
# 依次跑全部用例（自搜索 + 外部停靠点两种模式），每次一个日志序号，结果追加到 logs/汇总.md。
# 用法：./全部用例.sh [用例名关键字...] [--auto-only | --external-only | --modes=auto,external,external_no_yaw]
#   ./全部用例.sh               全部用例 × 自搜索、外部停靠点 2 种模式
#   ./全部用例.sh U形 T形        只跑名字里带 U形 或 T形 的
#   external_no_yaw：外部停靠点不带朝向，由找点节点定车头
# 每个用例都会重启整套仿真，单次约 3~5 分钟。中途 Ctrl+C 会停掉当前用例并退出。
ROOT="$(cd "$(dirname "$0")" && pwd)"
MODES=(auto external)
FILTERS=()
for arg in "$@"; do
  case "$arg" in
    --auto-only) MODES=(auto) ;;
    --external-only) MODES=(external) ;;
    --modes=*) IFS=, read -r -a MODES <<< "${arg#--modes=}" ;;
    *) FILTERS+=("$arg") ;;
  esac
done

mapfile -t ALL < <(python3 "$ROOT/容器/场景用例.py" --list | cut -f1)
SELECTED=()
for name in "${ALL[@]}"; do
  if [ ${#FILTERS[@]} -eq 0 ]; then SELECTED+=("$name"); continue; fi
  for f in "${FILTERS[@]}"; do [[ "$name" == *"$f"* ]] && { SELECTED+=("$name"); break; }; done
done
[ ${#SELECTED[@]} -gt 0 ] || { echo "没有匹配的用例"; exit 2; }

SUMMARY="$ROOT/logs/汇总.md"
mkdir -p "$ROOT/logs"
[ -f "$SUMMARY" ] || printf '| 时间 | 日志 | 用例 | 模式 | 结果 |\n| --- | --- | --- | --- | --- |\n' > "$SUMMARY"
stopped=0
trap 'stopped=1' INT
pass=0
fail=0
for name in "${SELECTED[@]}"; do
  for mode in "${MODES[@]}"; do
    [ "$stopped" = 1 ] && break 2
    args=(--no-hold)
    case "$mode" in
      auto) label=自搜索 ;;
      external) label=外部停靠点; args+=(--external) ;;
      external_no_yaw) label=外部停靠点无朝向; args+=(--external-no-yaw) ;;
      *) echo "未知模式：$mode"; exit 2 ;;
    esac
    tmp=$(mktemp)
    "$ROOT/运行用例.sh" "$name" "${args[@]}" < /dev/null 2>&1 | tee "$tmp"
    line=$(grep -E '^\[[0-9]{3,}_' "$tmp" | tail -1)
    rm -f "$tmp"
    id=$(printf '%s' "$line" | sed -n 's/^\[\([^]]*\)\].*/\1/p')
    result=$(printf '%s' "$line" | sed 's/^\[[^]]*\] //')
    [[ "$result" == PASS* ]] && pass=$((pass + 1)) || fail=$((fail + 1))
    printf '| %s | %s | %s | %s | %s |\n' "$(date '+%m-%d %H:%M')" "$id" "$name" \
      "$label" "${result//|//}" >> "$SUMMARY"
  done
done
echo "完成：PASS $pass，FAIL $fail。汇总：$SUMMARY"
