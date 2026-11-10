#!/bin/bash
# 一键避车仿真（容器内执行）。平时从 WSL 用 测试脚本/用例/*.sh 调用，它会负责日志编号和拷回。
#   bash /capella/避车测试/一键避车测试.sh --case 直线_左到右 [--external | --external-no-yaw] [--run-id 编号] [--no-hold]
# 先停掉之前的仿真，再从头启动。默认跑完保留画面，Ctrl+C 结束并清理；--no-hold 跑完直接清理退出。
set -eo pipefail
DIR=/capella/避车测试
PARAM=/capella/share/capella_ros_launcher/param/outdoor_cleaner_qp_2.yaml
CASE=原地图弧形_左到右
MODE=()
RUN_ID=""
HOLD=1
while [ $# -gt 0 ]; do
  case "$1" in
    --case) CASE="$2"; shift ;;
    --external|--external-no-yaw) MODE+=("$1") ;;
    --run-id) RUN_ID="$2"; shift ;;
    --no-hold) HOLD=0 ;;
    *) echo "未知参数：$1"; exit 2 ;;
  esac
  shift
done

source /opt/ros/humble/setup.bash
source /capella/setup.bash
source /root/avoid_ws/install/setup.bash

[ -n "$RUN_ID" ] || RUN_ID="$(TZ=Asia/Shanghai date +%Y%m%d_%H%M%S)_$CASE"
RUN="$DIR/logs/$RUN_ID"
INFO="$RUN/运行信息"
mkdir -p "$INFO"
ln -sfn "$RUN" "$DIR/logs/latest"
GROUPS_FILE="$INFO/process_groups"
: > "$GROUPS_FILE"
echo "用例 $CASE ${MODE[*]}，日志 $RUN"

# 按进程组停掉匹配的进程。不碰 PID 1 和本脚本自己的进程组。
stop_groups() {
  local groups="$1"
  [ -z "$groups" ] && return 0
  for g in $groups; do kill -INT -- "-$g" 2>/dev/null || true; done
  for _ in $(seq 1 20); do
    local alive=0
    for g in $groups; do
      if ps -eo pgid=,stat= | awk -v g="$g" '$1 == g && $2 !~ /^Z/ {f = 1} END {exit !f}'; then alive=1; fi
    done
    [ "$alive" = 0 ] && return 0
    sleep 0.5
  done
  for g in $groups; do kill -KILL -- "-$g" 2>/dev/null || true; done
}

stop_old_simulation() {
  local me pattern groups
  me=$(ps -o pgid= -p $$ | tr -d ' ')
  pattern='避车测试/一键避车测试|nav2_opt\.launch\.py|static_transform_publisher.*--child-frame-id odom|gzserver|gzclient|rviz2|cartographer_node|road_model_node|robot_avoidance|find_parking_space|welt_model|bt_garage_navigator|compute_right_edge_path|garage_navigator|避车测试/上位机场景'
  groups=$(ps -eo pgid=,stat=,args= | awk -v me="$me" -v pat="$pattern" \
    '$2 !~ /^Z/ && $1 != me && $1 > 1 && $0 ~ pat {print $1}' | sort -u | tr '\n' ' ')
  if [ -n "$groups" ]; then
    echo "停掉之前的仿真进程组：$groups"
    stop_groups "$groups"
  fi
}

# start <日志文件夹> <日志文件名> <命令...>：每个程序一个文件夹。
start() {
  local folder="$RUN/$1" name="$2"
  shift 2
  mkdir -p "$folder"
  setsid "$@" > "$folder/$name.log" 2>&1 < /dev/null &
  echo "$!" >> "$GROUPS_FILE"
}

restore_param() {
  if [ -f "$INFO/nav_param_before.yaml" ] && ! cmp -s "$INFO/nav_param_before.yaml" "$PARAM"; then
    cp "$INFO/nav_param_before.yaml" "$PARAM"
  fi
}

cleanup() {
  trap - EXIT INT TERM
  set +e
  echo
  echo "清理本次仿真……"
  stop_groups "$(tr '\n' ' ' < "$GROUPS_FILE")"
  restore_param
  echo "已停止。日志：$RUN"
}

wait_for() {
  local what="$1" seconds="$2"
  shift 2
  local end=$((SECONDS + seconds))
  until "$@" > /dev/null 2>&1; do
    if [ "$SECONDS" -ge "$end" ]; then
      echo "失败：${seconds} 秒内没有等到 $what，日志在 $RUN"
      return 1
    fi
    sleep 2
  done
}

lifecycle_active() {
  timeout 8 ros2 lifecycle get "$1" 2>/dev/null | grep -q '^active'
}

nav_ready() {
  lifecycle_active /bt_navigator && lifecycle_active /controller_server && lifecycle_active /behavior_server
}

stop_old_simulation
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

mkdir -p /tmp/avoid_field
cp "$DIR/测试场/avoid_field."* /tmp/avoid_field/
read -r SPAWN_X SPAWN_Y SPAWN_YAW TF_X TF_Y TF_YAW WORLD MAP < <(
  python3 "$DIR/上位机场景.py" --case "$CASE" --print-launch-args \
    --rviz-template "$DIR/避车测试.rviz" --rviz-out "$INFO/view.rviz")
[ -n "$MAP" ] || { echo "用例 $CASE 不存在"; exit 2; }

echo "[启动 1/3] nav2_opt.launch.py：Gazebo、完整导航、RViz"
cp "$PARAM" "$INFO/nav_param_before.yaml"
# 仿真 /odom 就是 Gazebo 世界坐标。容器里的 map.pbstream 不是仿真地图，
# Cartographer 在仿真里只会推算、越走越偏，所以不启动它，由这里发 Gazebo 到地图的固定 map -> odom。
start 定位_map_odom static_transform_publisher ros2 run tf2_ros static_transform_publisher \
  --x "$TF_X" --y "$TF_Y" --yaw "$TF_YAW" --frame-id map --child-frame-id odom
export ROBOT_LOCALIZER_MODE=3D
# launch 会用这个覆盖参数文件里的 max_vel_x 和平滑器上限。容器环境是 0.5，这里改成 1。
export MAX_LINEAR_VEL=1.0
# 这台容器的 Gazebo 订阅 /cmd_vel。compose 已设则沿用；未设时用 cmd_vel。
# 不要改成没人订阅的 cmd_vel_nav_ 或另一台机器上的 cmd_vel_nav。
if [ -z "${COLLISION_MONITOR_CMD_VEL_OUT_TOPIC:-}" ]; then
  export COLLISION_MONITOR_CMD_VEL_OUT_TOPIC=cmd_vel
fi
export COLLISION_MONITOR_CMD_VEL_OUT_TOPIC
NAV_ARGS=(x_pose:="$SPAWN_X" y_pose:="$SPAWN_Y" yaw:="$SPAWN_YAW" rviz_config_file:="$INFO/view.rviz")
if [ "$WORLD" != "-" ]; then
  export TB3_WORLD="$WORLD"
  NAV_ARGS+=(map:="$MAP")
fi
start 导航_nav2_opt nav2_opt ros2 launch capella_ros_launcher nav2_opt.launch.py "${NAV_ARGS[@]}"
wait_for "导航节点激活" 240 nav_ready
# launch 启动时会改写安装目录的导航参数文件，节点已经读完，立刻改回原样。
restore_param

echo "[启动 2/3] 道路拟合、通过性、找点、右边沿路径、贴边、避车行为树"
start 道路拟合_road_model_node road_model_node ros2 run garage_road_model road_model_node --ros-args \
  --params-file /root/avoid_ws/install/garage_road_model/share/garage_road_model/config/params.yaml \
  -p use_sim_time:=true
start 通过性_robot_avoidance robot_avoidance ros2 run robot_avoidance robot_avoidance --ros-args \
  --params-file /root/avoid_ws/install/robot_avoidance/share/robot_avoidance/params/config.yaml \
  -p use_sim_time:=true -p car_avoidance_demonstration:=false
# 停靠朝向是长边法线，车身垂直于长边，外侧距离要大于半个车长才能整车停出通道。
start 找点_find_parking_space find_parking_space ros2 run find_free_space find_parking_space --ros-args \
  --params-file /root/avoid_ws/install/find_free_space/share/find_free_space/params/config.yaml \
  -p use_sim_time:=true -p outside_min:=1.1 -p outside_max:=1.4
start 右边沿路径_compute_right_edge_path compute_right_edge_path ros2 run garage_utils \
  compute_right_edge_path_server_node --ros-args \
  --params-file /root/avoid_ws/install/garage_utils/share/garage_utils/params/config.yaml \
  -p use_sim_time:=true
start 贴边_welt_model welt_model ros2 run welt_model welt_model --ros-args -p use_sim_time:=true
sleep 3
start 避车行为树_garage_navigator garage_navigator ros2 launch garage_navigator navigation_launch.py use_sim_time:=true
wait_for "避车行为树激活" 120 lifecycle_active /bt_garage_navigator

echo "[启动 3/3] 模拟上位机"
mkdir -p "$RUN/上位机"
set +e
python3 "$DIR/上位机场景.py" --case "$CASE" "${MODE[@]}" --report "$RUN/上位机/report.json" 2>&1 \
  | tee "$RUN/上位机/上位机.log"
code=${PIPESTATUS[0]}
set -e
echo
if [ "$code" = 0 ]; then
  echo "结果：PASS。报告 $RUN/上位机/report.json"
else
  echo "结果：FAIL。报告 $RUN/上位机/report.json，各程序日志在 $RUN"
fi
if [ "$HOLD" = 1 ]; then
  echo "画面保留中，Ctrl+C 结束并清理。"
  while true; do sleep 1; done
fi
exit "$code"
