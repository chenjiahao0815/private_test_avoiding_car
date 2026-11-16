#!/usr/bin/env bash
# 在仿真容器的独立工作区编译当前本地源码，不使用测试脚本2的旧安装包。
set -eo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
PROJECT="$(cd "$ROOT/../../.." && pwd)"
SOURCE="$PROJECT/pkgs/avoiding_car"
CTR="${AVOID_TEST_CONTAINER:-capella-ros-core-1}"
WORKSPACE=/root/avoid_scene2_ws
LOG_ROOT="${AVOID_TEST_LOG_DIR:-/tmp/avoid_test2_logs}"
mkdir -p "$LOG_ROOT"
docker exec "$CTR" mkdir -p "$WORKSPACE/src"
for item in capella_msgs/capella_ros_msg capella_msgs/garage_utils_msgs \
  garage_avoidance_pkg/garage_behavior_tree garage_avoidance_pkg/garage_navigator \
  garage_avoidance_pkg/garage_utils find_free_space robot_avoidance welt_model; do
  package="${item##*/}"
  docker exec "$CTR" mkdir -p "$WORKSPACE/src/$package"
  tar -C "$SOURCE/$item" --exclude=.git --exclude=__pycache__ --exclude='*.log' \
    --exclude=build --exclude=install --exclude=log -cf - . \
    | docker exec -i "$CTR" tar -xf - -C "$WORKSPACE/src/$package"
done
# 避免旧快照中已被删除的 XML 留在本次源码或安装目录。
docker exec "$CTR" python3 -c '
from pathlib import Path
for base in (Path("/root/avoid_scene2_ws/src/garage_navigator/behavior_trees"),
             Path("/root/avoid_scene2_ws/install/garage_navigator/share/garage_navigator/behavior_trees")):
    for name in ("garage.xml", "garage2.xml", "garage_back.xml", "test.xml"):
        (base / name).unlink(missing_ok=True)
'
docker exec "$CTR" bash -c '
  set -eo pipefail
  source /opt/ros/humble/setup.bash
  source /capella/setup.bash
  cd /root/avoid_scene2_ws
  export MAKEFLAGS=-j2
  colcon build --base-paths src --executor sequential \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DPython3_EXECUTABLE=/usr/bin/python3
' 2>&1 | tee "$LOG_ROOT/编译.log"
