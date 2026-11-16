#!/usr/bin/env bash
# 默认编译当前源码并运行唯一场景。--no-build 复用本套工作区；--no-hold 结束后清理。
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
CTR="${AVOID_TEST_CONTAINER:-capella-ros-core-1}"
DEST=/capella/avoid_test2
BUILD=1
HOLD=0
for arg in "$@"; do
  case "$arg" in
    --no-build) BUILD=0 ;;
    --no-hold) HOLD=0 ;;
    *) echo "用法：$0 [--no-build] [--no-hold]"; exit 2 ;;
  esac
done
if [ "$BUILD" = 1 ]; then bash "$ROOT/编译.sh"; fi
ID="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_cars_gone"
LOG_ROOT="${AVOID_TEST_LOG_DIR:-/tmp/avoid_test2_logs}"
RUN="$LOG_ROOT/$ID"
mkdir -p "$RUN"
docker exec "$CTR" mkdir -p "$DEST/logs/$ID"
tar -C "$ROOT/容器" --exclude=__pycache__ -cf - . | docker exec -i "$CTR" tar -xf - -C "$DEST"
docker exec "$CTR" mkdir -p "$DEST/地图"
tar -C "$ROOT/地图" -cf - . | docker exec -i "$CTR" tar -xf - -C "$DEST/地图"
sync_logs() {
  # 容器有搬迁前的失效 bind mount 时 docker cp 会失败；从正在运行的容器内导出。
  docker exec "$CTR" tar -C "$DEST/logs/$ID" -cf - . 2>/dev/null \
    | tar --no-same-owner -xf - -C "$RUN" 2>/dev/null || true
}
stop_run() {
  docker exec "$CTR" python3 -c '
import os, signal, sys
from pathlib import Path
path = Path(sys.argv[1])
if path.exists():
    try: os.kill(int(path.read_text()), signal.SIGTERM)
    except ProcessLookupError: pass
' "$DEST/logs/$ID/运行信息/runner.pid" || true
}
trap 'stop_run' INT TERM
trap 'sync_logs' EXIT
echo "开始单场景测试，日志：$RUN"
docker exec "$CTR" bash "$DEST/运行场景.sh" "$ID" "$HOLD" &
runner=$!
while kill -0 "$runner" 2>/dev/null; do
  sync_logs
  sleep 3
done
set +e
wait "$runner"
code=$?
set -e
sync_logs
ln -sfn "$ID" "$LOG_ROOT/latest"
echo "本轮日志：$RUN"
exit "$code"
