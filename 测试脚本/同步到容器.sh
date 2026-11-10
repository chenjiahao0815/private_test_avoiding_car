#!/bin/bash
# 把 容器/ 目录同步到 capella 容器 /capella/避车测试/（不动容器里的 logs）。
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
CTR=capella-ros-core-1
DEST=/capella/避车测试

docker exec "$CTR" mkdir -p "$DEST/logs"
tar -C "$ROOT/容器" --exclude='__pycache__' -cf - . | docker exec -i "$CTR" tar -xf - -C "$DEST"
docker exec "$CTR" chmod +x "$DEST/一键避车测试.sh"
echo "已同步到 $CTR:$DEST"
