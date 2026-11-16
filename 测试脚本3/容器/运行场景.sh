#!/usr/bin/env bash
set -eo pipefail
DIR="$(cd "$(dirname "$0")" && pwd)"
ID="${1:?需要运行编号}"
HOLD="${2:-1}"
RUN="$DIR/logs/$ID"
# ASCII 路径传给 Nav2，避免旧 launch 把中文路径转义后写入参数。
FIELD="/tmp/avoid_scene3/$ID"
WS=/root/avoid_scene3_ws
export ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11365
export GARAGE_ENABLE_DUAL_SIDE_AVOIDANCE=true
mkdir -p "$RUN/运行信息" "$RUN/上位机" "$FIELD"
cat > "$FIELD/cyclonedds.xml" <<'XML'
<CycloneDDS><Domain id="any">
  <General><AllowMulticast>false</AllowMulticast></General>
  <Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    <Peers><Peer address="127.0.0.1"/></Peers>
  </Discovery>
</Domain></CycloneDDS>
XML
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$FIELD/cyclonedds.xml"
echo $$ > "$RUN/运行信息/runner.pid"
exec 8>/tmp/avoid_scene3.lock
flock -n 8 || { echo "测试脚本3已有一轮在运行"; exit 2; }
groups=()
cleanup() {
  trap - EXIT INT TERM
  set +e
  echo "清理本轮仿真……"
  for group in "${groups[@]}"; do kill -INT -- "-$group" 2>/dev/null; done
  for _ in {1..20}; do
    alive=0
    for group in "${groups[@]}"; do
      if ps -eo pgid=,stat= | awk -v g="$group" '$1==g && $2 !~ /^Z/ {f=1} END {exit !f}'; then alive=1; fi
    done
    [ "$alive" = 0 ] && break
    sleep 0.5
  done
  for group in "${groups[@]}"; do kill -KILL -- "-$group" 2>/dev/null; done
  wait 2>/dev/null
  cp -a "$FIELD/." "$RUN/运行信息/"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
start() {
  local folder="$RUN/$1" name="$2"
  shift 2
  mkdir -p "$folder"
  setsid "$@" > "$folder/$name.log" 2>&1 < /dev/null 8>&- &
  groups+=("$!")
  printf '%s\n' "$!" >> "$RUN/运行信息/process_groups"
}
active() {
  timeout 8 ros2 lifecycle get "$1" 2>/dev/null | grep -q '^active'
}
wait_active() {
  local end=$((SECONDS+240))
  until active "$1"; do
    if ((SECONDS>=end)); then echo "等待 $1 激活超时"; exit 1; fi
    sleep 2
  done
}
source /opt/ros/humble/setup.bash
source /capella/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true
python3 "$DIR/生成场景.py" --out "$FIELD" \
  --params /capella/share/capella_ros_launcher/param/outdoor_cleaner_qp_2.yaml \
  --rviz-template "$DIR/避车测试.rviz"
# nav2_opt 原本会改写安装目录参数；本轮传入自己的副本，并关闭该改写分支。
export MAX_LINEAR_VEL="" MAX_LINEAR_ACC="" IGNORE_HEIGHT_ABOVE=""
export ROBOT_LOCALIZER_MODE=3D GAZEBO_OR_FAKE=gazebo
export TB3_WORLD="$FIELD/scene.world"
start 定位_map_odom static_transform_publisher ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --yaw 0 --frame-id map --child-frame-id odom
start 导航_nav2_opt nav2_opt ros2 launch capella_ros_launcher nav2_opt.launch.py \
  x_pose:=43.5 y_pose:=19.75 yaw:=3.141592653589793 headless:=True \
  map:="$FIELD/scene.yaml" params_file:="$FIELD/nav_params.yaml" rviz_config_file:="$FIELD/view.rviz"
wait_active /bt_navigator
wait_active /controller_server
wait_active /behavior_server

# Nav2 使用容器自带接口；避车节点使用本次编译的接口和程序。
source "$WS/install/local_setup.bash"
python3 "$DIR/场景测试.py" --check-interface | tee "$RUN/运行信息/interface_check.log"
start 通过性_robot_avoidance robot_avoidance ros2 run robot_avoidance robot_avoidance --ros-args \
  --params-file "$WS/install/robot_avoidance/share/robot_avoidance/params/config.yaml" \
  -p use_sim_time:=true -p car_avoidance_demonstration:=false
start 右边沿路径_compute_right_edge_path compute_right_edge_path ros2 run garage_utils compute_right_edge_path_server_node --ros-args \
  --params-file "$WS/install/garage_utils/share/garage_utils/params/config.yaml" -p use_sim_time:=true
start 贴边_welt_model welt_model ros2 run welt_model welt_model --ros-args -p use_sim_time:=true
start 找点_find_parking_space find_parking_space ros2 run find_free_space find_parking_space --ros-args \
  --params-file "$WS/install/find_free_space/share/find_free_space/params/config.yaml" \
  -p use_sim_time:=true -p outside_min:=1.1 -p outside_max:=1.4 -p enable_dual_side:=true
start 避车行为树_garage_navigator garage_navigator ros2 launch garage_navigator navigation_launch.py use_sim_time:=true
wait_active /bt_garage_navigator
value=$(timeout 15 ros2 param get /bt_garage_navigator use_sim_time)
printf '%s %s\n' /bt_garage_navigator "$value" | tee "$RUN/运行信息/clock_check.log"
[[ "$value" == *True* ]] || { echo "行为树没有使用仿真时钟"; exit 1; }
# Nav2 内部 client node 不持续 spin 默认参数服务，用实际 /clock 订阅验证它的时钟。
timeout 15 ros2 topic info /clock --verbose > "$RUN/运行信息/clock_subscribers.log"
grep -q 'Node name: bt_garage_navigator_garage_vehicle_avoidance_rclcpp_node' "$RUN/运行信息/clock_subscribers.log" \
  || { echo "内部行为树节点未订阅 /clock"; exit 1; }
start 上位机 上位机 python3 -u "$DIR/场景测试.py" --report "$RUN/上位机/report.json"
test_pid="${groups[-1]}"
set +e
wait "$test_pid"
code=$?
set -e
cat "$RUN/上位机/上位机.log"
printf '%s\n' "$code" > "$RUN/运行信息/completed"
cp -a "$FIELD/." "$RUN/运行信息/"
if [ "$HOLD" = 1 ]; then
  echo "结果和画面已保留，Ctrl+C 清理。日志：$RUN"
  while true; do sleep 1; done
fi
exit "$code"
