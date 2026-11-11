#!/usr/bin/env python3
"""模拟上位机走一遍双侧避车：定位、发通道、发禁扫区、行驶、来车、查通过性、避车动作、车走后恢复行驶。

和 测试脚本/ 的区别：
  - 只有矩形通道，只测自搜索停靠点；
  - 按用例在 /cleaning_tool_retraction_areas 发禁扫区，把近侧长边外侧堵住；
  - 按用例在 /car_information_all 发远侧动态车（空 / 一直有 / 断续），看到靠右路径后放行；
  - 停好后从行为树日志里核对 3 秒、4 秒、15 秒、5 秒这些时间。
"""

import argparse
import json
import math
import os
import re
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import 场景用例 as cases  # noqa: E402

HALF_WIDTH = cases.HALF_WIDTH
ROBOT_LANE = cases.ROBOT_LANE  # 机器人离右侧长边的距离
CAR_LANE = 0.45            # 来车偏离中心线、靠左侧（对机器人而言）
CAR_SIZE = (4.5, 1.8, 1.5)
CAR_SPEED = 1.5
START_S = 1.5
END_MARGIN = 1.5
GOAL_SPACING = 2.0
POSITION_TOLERANCE = 0.35
YAW_TOLERANCE = 0.2
MAX_OFF_TANGENT_DEG = 35
FLICKER_PERIOD = 2.0       # 断续动态车：有车 2 秒、空 2 秒
DEFAULT_CASE = "矩形_远侧无车"

# 行为树日志里的关键事件。
BT_EVENTS = {
    "far_start": "记下远侧等待起点",
    "far_found": "找点返回 side=2",
    "clear": "秒没有动态车，直接使用已保存的远侧点",
    "approve": "批准停靠点",
    "busy": "秒检测到动态车",
    "timeout": "远侧等待达到",
    "commit": "进入靠右路径",
    "arm": "靠右跟踪开始",
    "unsuppress": "远侧屏蔽结束",
    "drop": "远侧结果在屏蔽期内",
}
LOG_LINE = re.compile(r"\[(INFO|WARN|ERROR)\] \[(\d+\.\d+)\] \[[^\]]+\]: (.*)")


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def read_events(path):
    """[(墙钟秒, 事件名, 原文)]，按时间排序。"""
    events = []
    try:
        lines = open(path, errors="replace").read().splitlines()
    except OSError:
        return events
    for line in lines:
        m = LOG_LINE.search(line)
        if not m:
            continue
        stamp, text = float(m.group(2)), m.group(3)
        for name, key in BT_EVENTS.items():
            if key in text:
                events.append((stamp, name, text))
    events.sort(key=lambda e: e[0])
    return events



def event_point(text):
    match = re.search(r"\(([-+\d.eE]+),\s*([-+\d.eE]+)\)", text)
    return tuple(map(float, match.groups())) if match else None


def evaluate_timeline(events, expect, dual, has_right_edge, find_text, bt_text):
    """按保存远侧点、等待后直接放行的规则检查日志，不要求再次找点。"""
    first = {}
    for stamp, name, _ in events:
        first.setdefault(name, stamp)
    problems = []
    approvals = [event for event in events if event[1] == "approve"]
    approval = approvals[-1] if approvals else None
    approve_side = None
    if approval:
        match = re.search(r"side=(\d+)", approval[2])
        approve_side = int(match.group(1)) if match else None

    def last_before(name, stamp, since=float("-inf")):
        return next((event for event in reversed(events)
                     if event[1] == name and since <= event[0] <= stamp), None)

    def require_events(*names):
        missing = [name for name in names if name not in first]
        problems.extend(f"行为树日志里没有 {BT_EVENTS[name]}" for name in missing)
        return not missing

    if dual and not events:
        problems.append("没有读取到本轮行为树事件，检查日志路径和节点运行情况")
    if expect["approve_side"] != "any" and approve_side != expect["approve_side"]:
        problems.append(f"批准的 side 应为 {expect['approve_side']}，实际 {approve_side}")
    if expect["park"] in ("left", "right") and not approval:
        problems.append("应停车却没有批准停靠点")
    if expect["park"] == "none" and approval:
        problems.append("期望不停靠，却批准了停靠点")
    if expect["far_wait"] is True and "far_start" not in first:
        problems.append("应该进入远侧等待却没有")
    if expect["far_wait"] is False and "far_start" in first:
        problems.append("不应进入远侧等待却进入了")
    if expect["right_edge"] != has_right_edge:
        problems.append("应该走靠右却没有靠右路径" if expect["right_edge"]
                        else "不应走靠右却走了")

    if approval and approve_side == 2:
        stamp = approval[0]
        start = last_before("far_start", stamp)
        commit = last_before("commit", stamp)
        if start is None or (commit and start[0] < commit[0]):
            problems.append("批准远侧点之前没有本轮远侧等待起点")
        else:
            wait = stamp - start[0]
            if wait < expect["min_wait"] - 0.3:
                problems.append(f"远侧点等了 {wait:.1f} 秒就批准，应 ≥{expect['min_wait']} 秒")
            if wait >= 15.0 * 2.5 + 2.0:
                problems.append(f"远侧点等待 {wait:.1f} 秒仍被批准，超过总等待上限")
            clear = last_before("clear", stamp, start[0])
            if clear is None:
                problems.append("批准远侧点前没有连续无车放行事件")
            if any(start[0] <= t <= stamp and name in ("busy", "timeout")
                   for t, name, _ in events):
                problems.append("本轮远侧已被放弃，却仍批准了该点")
            found = last_before("far_found", start[0])
            found_point = event_point(found[2]) if found else None
            approved_point = event_point(approval[2])
            if found_point is None or approved_point is None:
                problems.append("日志缺少首次远侧点或批准点坐标")
            elif math.dist(found_point, approved_point) > 0.02:
                problems.append(f"批准点 {approved_point} 与首次保存的远侧点 {found_point} 不一致")
            for line in find_text.splitlines():
                match = LOG_LINE.search(line)
                if match and start[0] < float(match.group(2)) < stamp and \
                        "开始寻找避让点" in match.group(3):
                    problems.append("保存远侧点后等待期间仍重新调用 find")
                    break

    if expect["give_up"] in ("busy", "timeout"):
        reason = expect["give_up"]
        if require_events("far_start", reason):
            event = next(event for event in events if event[1] == reason)
            start = last_before("far_start", event[0])
            if start is None:
                problems.append("远侧放弃事件之前没有等待起点")
            else:
                wait = event[0] - start[0]
                seconds = 4.0 if reason == "busy" else 15.0
                if not seconds - 0.3 <= wait <= seconds * 2.5 + 2.0:
                    problems.append(f"远侧放弃用了 {wait:.1f} 秒，应约 {seconds:g} 秒")
            other = "timeout" if reason == "busy" else "busy"
            if other in first and first[other] < event[0]:
                problems.append(f"远侧应由 {reason} 放弃，却先出现 {other}")
    elif expect["give_up"] is None and ("busy" in first or "timeout" in first):
        problems.append("不应放弃远侧却放弃了")

    if dual:
        for stamp, name, _ in events:
            if name in ("arm", "unsuppress"):
                commit = last_before("commit", stamp)
                if commit is None:
                    problems.append(f"{name} 之前没有进入靠右事件")
                    continue
                if name == "unsuppress":
                    arm = last_before("arm", stamp, commit[0])
                    if arm is None:
                        problems.append("靠右跟踪尚未开始就解除远侧屏蔽")
                    elif stamp - arm[0] < 5.0 - 0.3:
                        problems.append(f"靠右跟踪 {stamp - arm[0]:.1f} 秒就解除远侧屏蔽，应 ≥5 秒")
            elif name == "far_start":
                commit = last_before("commit", stamp)
                if commit:
                    arm = last_before("arm", stamp, commit[0])
                    if arm is None or stamp < arm[0] + 5.0 - 0.3:
                        problems.append("靠右规划期间或 5 秒保护期内就采纳了远侧点")
    else:
        if "GARAGE_ENABLE_DUAL_SIDE_AVOIDANCE=false" not in bt_text or "dual_side.xml" in bt_text:
            problems.append("双侧开关关了，行为树却没有退回老 XML")
        if "找点结果 side=2" in find_text or "far_found" in first:
            problems.append("双侧开关关了，找点却返回了远侧点")

    if expect["find_log"] and expect["find_log"] not in find_text:
        problems.append(f"找点日志里没有“{expect['find_log']}”")
    return {"first": first, "approve_side": approve_side, "problems": list(dict.fromkeys(problems))}


def check_interface():
    """在启动仿真前验证实际导入的接口和序列化。"""
    import capella_ros_msg
    from capella_ros_msg.action import FindCarAvoidancePoint
    from rclpy.serialization import deserialize_message, serialize_message

    goal_fields = FindCarAvoidancePoint.Goal.get_fields_and_field_types()
    result_fields = FindCarAvoidancePoint.Result.get_fields_and_field_types()
    if "enable_far_side" not in goal_fields or "side" not in result_fields:
        raise RuntimeError(
            f"加载了旧找点接口 {capella_ros_msg.__file__}，缺少 enable_far_side 或 side。"
            "请先加载 avoid_msg_ws 新接口，再重新编译 avoid_ws，不能继续双侧测试。")
    goal = FindCarAvoidancePoint.Goal()
    goal.enable_far_side = True
    result = FindCarAvoidancePoint.Result()
    result.side = 2
    if not deserialize_message(serialize_message(goal), type(goal)).enable_far_side or \
            deserialize_message(serialize_message(result), type(result)).side != 2:
        raise RuntimeError("找点接口序列化失败")
    print(f"接口检查通过：{capella_ros_msg.__file__}，enable_far_side=True，side=2", flush=True)


def review_log_dir(path):
    """只重评已有日志的时间线，保留历史 report.json 和完整流程结论。"""
    with open(os.path.join(path, "上位机", "report.json")) as handle:
        report = json.load(handle)
    case = cases.CASES.get(report["case"])
    expect = case["expect"] if case else report["expect"]
    dual = case["dual"] if case else report.get("dual", True)
    bt_path = os.path.join(path, "避车行为树_garage_navigator", "garage_navigator.log")
    find_path = os.path.join(path, "找点_find_parking_space", "find_parking_space.log")
    with open(bt_path, errors="replace") as handle:
        bt_text = handle.read()
    with open(find_path, errors="replace") as handle:
        find_text = handle.read()
    begin = next((float(match.group(2)) for line in bt_text.splitlines()
                  if (match := LOG_LINE.search(line)) and
                  "Begin to execute garage_vehicle_avoidance program." in match.group(3)), None)
    if begin is None:
        raise RuntimeError("日志没有本轮避车开始时间")
    events = [event for event in read_events(bt_path) if event[0] >= begin]
    checked = evaluate_timeline(events, expect, dual, bool(report.get("靠右")),
                                find_text, bt_text)
    checked["first"] = {name: round(stamp - begin, 2) for name, stamp in checked["first"].items()}
    checked["时间线判定"] = "FAIL" if checked["problems"] else "PASS"
    checked["原始完整流程结果"] = report.get("result", "未完成")
    print(json.dumps(checked, ensure_ascii=False, indent=2))
    return 1 if checked["problems"] else 0

class Scene:
    """一个用例的几何：通道多边形、机器人车道、来车车道、禁扫区。"""

    def __init__(self, name):
        if name not in cases.CASES:
            raise SystemExit(f"没有用例 {name}，可选：{', '.join(cases.CASES)}")
        self.case = cases.CASES[name]
        self.name = name
        self.path = self.case["path"]
        self.polygon = self.case["polygon"]
        self.no_go = list(self.case["no_go"])
        self.timeline = self.case["timeline"]
        self.expect = self.case["expect"]
        self.dual = self.case["dual"]
        self.world = cases.WORLDS[self.case["world"]]
        self.end_s = self.path.length - END_MARGIN

    def lane(self, s):
        sign = 1.0 if self.case.get("hug") == "left" else -1.0
        return self.path.offset(s, sign * (HALF_WIDTH - ROBOT_LANE))

    def car_at(self, s):
        sign = -1.0 if self.case.get("hug") == "left" else 1.0
        x, y, heading = self.path.offset(s, sign * CAR_LANE)
        return x, y, heading + math.pi

    def corridor_heading(self, x, y, near_s, window):
        s, _ = self.path.project(x, y, near_s - window, near_s + window)
        return self.path.at(s)[2]

    def inside(self, x, y):
        return cases._inside((x, y), self.polygon)

    def gazebo_to_map(self, x, y, yaw):
        dx, dy, rot = self.world["map_from_gazebo"]
        c, s = math.cos(rot), math.sin(rot)
        return c * x - s * y + dx, s * x + c * y + dy, yaw + rot

    def map_to_gazebo(self, x, y, yaw):
        dx, dy, rot = self.world["map_from_gazebo"]
        c, s = math.cos(rot), math.sin(rot)
        x, y = x - dx, y - dy
        return c * x + s * y, -s * x + c * y, yaw - rot

    def bounds(self):
        xs = [p[0] for p in self.polygon]
        ys = [p[1] for p in self.polygon]
        names = set(self.no_go) | {n for t in self.timeline if t[2] == "no_go" for n in t[3]}
        for n in names:
            xs += [p[0] for p in cases.NO_GO[n]]
            ys += [p[1] for p in cases.NO_GO[n]]
        return min(xs), min(ys), max(xs), max(ys)

    def describe(self, x, y, robot_s):
        """停靠点相对通道的位置：在机器人前/后多少米，在行驶方向左/右多少米，是否在通道里。"""
        s, _ = self.path.project(x, y, robot_s - 12.0, robot_s + 12.0)
        px, py, heading = self.path.at(s)
        lateral = -math.sin(heading) * (x - px) + math.cos(heading) * (y - py)
        return {
            "along": round(s - robot_s, 2),
            "lateral": round(lateral, 2),
            "side": "左" if lateral > 0 else "右",
            "in_corridor": self.inside(x, y),
        }

    def write_rviz(self, template, out):
        x0, y0, x1, y1 = self.bounds()
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        scale = max(8.0, min(40.0, 800.0 / max(x1 - x0 + 6.0, 1.0), 560.0 / max(y1 - y0 + 6.0, 1.0)))
        lines = open(template).read().split("\n")
        start = next(i for i, line in enumerate(lines) if "rviz_default_plugins/TopDownOrtho" in line)
        done = set()
        for i in range(start, min(start + 25, len(lines))):
            for key, value in (("Scale", scale), ("X", cx), ("Y", cy), ("Angle", 0.0)):
                if key not in done and re.match(rf"^\s+{key}: ", lines[i]):
                    lines[i] = re.sub(rf"{key}: .*", f"{key}: {value:.3f}", lines[i])
                    done.add(key)
        with open(out, "w") as f:
            f.write("\n".join(lines))

    def launch_args(self):
        x, y, yaw = self.map_to_gazebo(*self.lane(START_S))
        dx, dy, rot = self.world["map_from_gazebo"]
        return [f"{x:.3f}", f"{y:.3f}", f"{yaw:.4f}", str(dx), str(dy), str(rot),
                self.world["world"] or "-", self.world["map"] or "-", "1" if self.dual else "0"]


def main_prepare(args):
    scene = Scene(args.case)
    if args.rviz_template and args.rviz_out:
        scene.write_rviz(args.rviz_template, args.rviz_out)
    print(" ".join(scene.launch_args()))


# ---------------------------------------------------------------- ROS 部分
def run_ros(args):
    import rclpy
    from action_msgs.msg import GoalStatus
    from gazebo_msgs.srv import GetEntityState
    from geometry_msgs.msg import Point, Point32, Polygon, PoseArray, PoseStamped, TwistStamped
    from nav2_msgs.action import NavigateThroughPoses
    from nav_msgs.msg import Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    from capella_ros_msg.msg import CarDetectArray, CarDetectSingle
    from capella_ros_msg.srv import IsCarPassable
    from garage_utils_msgs.action import GarageVehicleAvoidance
    from garage_utils_msgs.msg import Polygons

    def stamped(node, x, y, yaw, z=0.0):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    class Fail(Exception):
        pass

    class UpperComputer(Node):
        def __init__(self, scene, report_path):
            super().__init__("avoid_test_upper", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.scene = scene
            self.report_path = report_path
            run_dir = os.path.dirname(os.path.dirname(os.path.abspath(report_path)))
            self.bt_log = os.path.join(run_dir, "避车行为树_garage_navigator", "garage_navigator.log")
            self.find_log = os.path.join(run_dir, "找点_find_parking_space", "find_parking_space.log")
            self.report = {"case": scene.name, "title": scene.case["title"], "expect": scene.expect,
                           "no_go": scene.no_go, "timeline": scene.timeline, "dual": scene.dual}
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.polygon_pub = self.create_publisher(Polygons, "/garage_polygons", latched)
            self.no_go_pub = self.create_publisher(Polygons, "/cleaning_tool_retraction_areas", latched)
            self.stop_points_pub = self.create_publisher(PoseArray, "/vehicle_avoidance_stop_points", latched)
            self.car_pub = self.create_publisher(CarDetectArray, "/car_information", 10)
            self.dynamic_pub = self.create_publisher(CarDetectArray, "/car_information_all", 10)
            self.marker_pub = self.create_publisher(MarkerArray, "/avoid_test/markers", latched)
            self.parking = None
            self.right_edge_path = None
            self.right_edge_wall = None
            self.create_subscription(Marker, "/marker_parking_point", self.on_parking, latched)
            self.create_subscription(NavPath, "/garage_path", self.on_right_edge, 10)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.passable = self.create_client(IsCarPassable, "/check_car_passable")
            self.gazebo_state = self.create_client(GetEntityState, "/get_entity_state")
            self.nav = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
            self.avoid_client = ActionClient(self, GarageVehicleAvoidance, "/garage_vehicle_avoidance")
            self.polygons = None
            self.car = None
            self.car_visible = False
            self.dynamic_mode = "none"
            self.dynamic_since = time.monotonic()
            self.dynamic_present = False
            self.no_go_names = list(scene.no_go)
            self.trigger_wall = {}
            self.timeline_done = set()
            self.parking_since = None
            self.status = "starting"
            self.publish_no_go()
            self.create_timer(0.1, self.publish_cars)
            self.create_timer(0.5, self.publish_markers)

        # ---------- 通用 ----------
        def on_parking(self, msg):
            if self.parking_since is None:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp + 1e-3 >= self.parking_since:
                self.parking = msg
                self.trigger_wall.setdefault("point", time.time())

        def on_right_edge(self, msg):
            if msg.poses and self.parking_since is not None:
                self.right_edge_path = msg
                if self.right_edge_wall is None:
                    self.right_edge_wall = time.time()
                    self.trigger_wall.setdefault("right_edge", self.right_edge_wall)

        def log(self, step, text, **data):
            line = f"[{step}] {text}"
            if data:
                line += "  " + json.dumps(data, ensure_ascii=False)
            print(line, flush=True)
            if data:
                self.report[step] = data
            self.save()

        def save(self):
            temporary = self.report_path + ".tmp"
            with open(temporary, "w") as handle:
                json.dump(self.report, handle, ensure_ascii=False, indent=2)
            os.replace(temporary, self.report_path)

        def spin_for(self, seconds):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)

        def wait_until(self, predicate, seconds, what):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
                result = predicate()
                if result:
                    return result
            raise Fail(f"{seconds:.0f} 秒内没有等到：{what}")

        def sim_now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def robot(self):
            try:
                t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time()).transform
            except Exception:
                return None
            return t.translation.x, t.translation.y, yaw_of(t.rotation)

        def gazebo_pose(self):
            if not self.gazebo_state.wait_for_service(timeout_sec=5.0):
                return None
            request = GetEntityState.Request()
            request.name = "turtlebot3_waffle"
            request.reference_frame = "world"
            future = self.gazebo_state.call_async(request)
            self.wait_until(future.done, 5.0, "Gazebo 机器人真值")
            state = future.result()
            if not state.success:
                return None
            p = state.state.pose
            return self.scene.gazebo_to_map(p.position.x, p.position.y, yaw_of(p.orientation))

        def localization_error(self):
            pose = self.robot()
            truth = self.gazebo_pose()
            if pose is None or truth is None:
                return None
            return math.hypot(pose[0] - truth[0], pose[1] - truth[1])

        def wait_still(self, seconds=1.0, timeout=20.0):
            last = None
            since = None
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                self.spin_for(0.2)
                pose = self.robot()
                if pose is None:
                    continue
                if last and math.hypot(pose[0] - last[0], pose[1] - last[1]) < 0.01 and \
                        angle_diff(pose[2], last[2]) < 0.01:
                    since = since or time.monotonic()
                    if time.monotonic() - since >= seconds:
                        return pose
                else:
                    since = None
                last = pose
            raise Fail("机器人一直没有停稳")

        def progress(self, lo, hi):
            pose = self.robot()
            if pose is None:
                return None
            s, _ = self.scene.path.project(pose[0], pose[1], lo, hi)
            return s

        # ---------- 发布 ----------
        def publish_no_go(self):
            areas = Polygons()
            for name in self.no_go_names:
                polygon = Polygon()
                polygon.points = [Point32(x=float(x), y=float(y)) for x, y in cases.NO_GO[name]]
                areas.polygons.append(polygon)
            self.no_go_pub.publish(areas)

        def run_timeline(self):
            now = time.time()
            for i, (trigger, delay, action, value) in enumerate(self.scene.timeline):
                if i in self.timeline_done or trigger not in self.trigger_wall:
                    continue
                if now < self.trigger_wall[trigger] + delay:
                    continue
                self.timeline_done.add(i)
                if action == "dynamic":
                    self.set_dynamic(value)
                else:
                    self.no_go_names = list(value)
                    self.publish_no_go()
                    print(f"[禁扫区] 换成 {self.no_go_names or '无'}", flush=True)
                self.report.setdefault("时间线动作", []).append(
                    [round(now - self.trigger_wall.get("avoid", now), 2), trigger, action, value])

        def set_dynamic(self, mode):
            if mode != self.dynamic_mode:
                self.dynamic_mode = mode
                self.dynamic_since = time.monotonic()
                print(f"[动态车] /car_information_all 切到 {mode}", flush=True)

        def car_message(self):
            array = CarDetectArray()
            if self.car_visible and self.car:
                single = CarDetectSingle()
                single.pose = stamped(self, self.car[0], self.car[1], self.car[2], z=100.0)
                single.size.x, single.size.y, single.size.z = CAR_SIZE
                single.speed = TwistStamped()
                single.speed.header.frame_id = "base_link"
                array.results.append(single)
            return array

        def dynamic_message(self):
            if self.dynamic_mode == "busy":
                present = True
            elif self.dynamic_mode == "flicker":
                phase = (time.monotonic() - self.dynamic_since) % (2.0 * FLICKER_PERIOD)
                present = phase < FLICKER_PERIOD
            else:
                present = False
            self.dynamic_present = present
            array = CarDetectArray()
            if present:
                # 远侧长边外侧来往的车，位置只用于画图，行为树只看有没有。
                x0, _, x1, y1 = self.scene.bounds()
                x = x0 + (time.monotonic() * 3.0) % max(x1 - x0, 1.0)
                single = CarDetectSingle()
                single.pose = stamped(self, x, y1 + 2.5, 0.0, z=100.0)
                single.size.x, single.size.y, single.size.z = CAR_SIZE
                single.speed = TwistStamped()
                single.speed.header.frame_id = "map"
                single.speed.twist.linear.x = 3.0
                array.results.append(single)
            return array

        def publish_cars(self):
            self.car_pub.publish(self.car_message())
            self.run_timeline()
            message = self.dynamic_message()
            if self.dynamic_mode != "silent":
                self.dynamic_pub.publish(message)

        def publish_markers(self):
            stamp = self.get_clock().now().to_msg()
            markers = MarkerArray()

            def base(ns, ident, kind):
                m = Marker()
                m.header.frame_id = "map"
                m.header.stamp = stamp
                m.ns = ns
                m.id = ident
                m.type = kind
                m.action = Marker.ADD
                m.pose.orientation.w = 1.0
                return m

            outline = base("corridor", 0, Marker.LINE_STRIP)
            outline.scale.x = 0.06
            outline.color.r, outline.color.g, outline.color.b, outline.color.a = 1.0, 0.85, 0.1, 1.0
            points = list(self.scene.polygon)
            outline.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in points + points[:1]]
            markers.markers.append(outline)

            for ident, name in enumerate(sorted(cases.NO_GO)):
                no_go = base("no_go", 10 + ident, Marker.LINE_STRIP)
                if name in self.no_go_names:
                    no_go.scale.x = 0.08
                    no_go.color.r, no_go.color.g, no_go.color.b, no_go.color.a = 0.9, 0.2, 0.9, 1.0
                    pts = list(cases.NO_GO[name])
                    no_go.points = [Point(x=float(x), y=float(y), z=0.06) for x, y in pts + pts[:1]]
                else:
                    no_go.action = Marker.DELETE
                markers.markers.append(no_go)

            lane = base("lane", 6, Marker.LINE_STRIP)
            lane.scale.x = 0.03
            lane.color.r = lane.color.g = lane.color.b = 0.5
            lane.color.a = 0.6
            s_values = [0.5 * k for k in range(int(self.scene.path.length / 0.5) + 1)]
            lane.points = [Point(x=float(p[0]), y=float(p[1]), z=0.04)
                           for p in (self.scene.lane(s) for s in s_values)]
            markers.markers.append(lane)

            car = base("car", 1, Marker.CUBE)
            car_text = base("car", 2, Marker.TEXT_VIEW_FACING)
            if self.car and self.car_visible:
                car.pose = stamped(self, self.car[0], self.car[1], self.car[2]).pose
                car.pose.position.z = 0.75
                car.scale.x, car.scale.y, car.scale.z = CAR_SIZE
                car.color.r, car.color.g, car.color.b, car.color.a = 0.9, 0.1, 0.1, 0.75
                car_text.pose.position.x = self.car[0]
                car_text.pose.position.y = self.car[1]
                car_text.pose.position.z = 2.0
                car_text.scale.z = 0.6
                car_text.color.r, car_text.color.a = 0.8, 1.0
                car_text.text = "CAR"
            else:
                car.action = Marker.DELETE
                car_text.action = Marker.DELETE
            markers.markers.extend([car, car_text])

            goal = base("final_goal", 3, Marker.ARROW)
            goal.pose = stamped(self, *self.scene.lane(self.scene.end_s)).pose
            goal.scale.x, goal.scale.y, goal.scale.z = 1.0, 0.2, 0.2
            goal.color.g, goal.color.a = 0.9, 1.0
            markers.markers.append(goal)

            x0, y0, x1, y1 = self.scene.bounds()
            status = base("status", 4, Marker.TEXT_VIEW_FACING)
            status.pose.position.x = (x0 + x1) / 2.0
            status.pose.position.y = y1 + 4.5
            status.pose.position.z = 1.0
            status.scale.z = 0.9
            status.color.b, status.color.a = 0.6, 1.0
            status.text = f"{self.scene.case['label']}   {self.status}"
            markers.markers.append(status)

            dyn = base("dynamic", 5, Marker.TEXT_VIEW_FACING)
            dyn.pose.position.x = (x0 + x1) / 2.0
            dyn.pose.position.y = y1 + 3.3
            dyn.pose.position.z = 1.0
            dyn.scale.z = 0.7
            dyn.color.r, dyn.color.g, dyn.color.a = (0.9, 0.2, 1.0) if self.dynamic_present else (0.1, 0.7, 1.0)
            dyn.color.b = 0.1
            dyn.text = f"/car_information_all: {self.dynamic_mode} ({'CAR' if self.dynamic_present else 'empty'})"
            markers.markers.append(dyn)
            self.marker_pub.publish(markers)

        # ---------- 步骤 ----------
        def localize(self):
            self.status = "1/7 localization"
            self.wait_until(lambda: self.robot() is not None, 120.0, "map -> base_link")
            pose = self.wait_still(2.0)
            start = self.scene.lane(START_S)
            error = self.localization_error()
            if error is None or error > 0.1:
                raise Fail(f"TF 定位和 Gazebo 真值差 {error} m，map -> odom 不是本脚本发的")
            if math.hypot(pose[0] - start[0], pose[1] - start[1]) > 0.3 or angle_diff(pose[2], start[2]) > 0.25:
                raise Fail(f"机器人不在起点：{[round(v, 3) for v in pose]}，应在 {[round(v, 3) for v in start]}")
            self.log("定位", "机器人在通道起点", map_pose=[round(v, 3) for v in pose], gazebo_error=round(error, 3))

        def build_corridor(self):
            self.status = "2/7 corridor"
            polygon = Polygon()
            polygon.points = [Point32(x=float(x), y=float(y)) for x, y in self.scene.polygon]
            self.polygons = Polygons()
            self.polygons.polygons.append(polygon)
            self.polygon_pub.publish(self.polygons)
            self.publish_no_go()
            # 无曲线版本的通过性、找点直接用通道四个顶点，不需要道路拟合。
            self.spin_for(1.0)
            self.log("通道", "已发布矩形通道和禁扫区", vertices=len(polygon.points), no_go=self.no_go_names)

        def send_lane(self, s_from):
            if not self.nav.wait_for_server(timeout_sec=30.0):
                raise Fail("没有 /navigate_through_poses")
            s_values = []
            s = s_from
            while s < self.scene.end_s - 0.5:
                s_values.append(s)
                s += GOAL_SPACING
            s_values.append(self.scene.end_s)
            goal = NavigateThroughPoses.Goal()
            goal.poses = [stamped(self, *self.scene.lane(s)) for s in s_values]
            future = self.nav.send_goal_async(goal)
            handle = self.wait_until(future.done, 15.0, "导航接收") and future.result()
            if not handle.accepted:
                raise Fail("导航目标被拒绝")
            return handle

        def drive_until_car(self):
            self.status = "3/7 driving in lane"
            trigger = self.scene.case["trigger_s"]
            handle = self.send_lane(START_S + GOAL_SPACING)
            result = handle.get_result_async()
            side_name = "左侧" if self.scene.case.get("hug") == "left" else "右侧"
            self.log("行驶", f"沿通道{side_name}车道向前开", trigger_s=trigger, length=round(self.scene.path.length, 1))
            last = [START_S]

            def reached():
                if result.done():
                    return True
                s = self.progress(last[0] - 1.0, last[0] + 4.0)
                if s is not None:
                    last[0] = max(last[0], s)
                return last[0] >= trigger

            self.wait_until(reached, 240.0, f"机器人开到弧长 s={trigger}")
            if result.done():
                raise Fail(f"还没遇到来车导航就结束了，状态 {result.result().status}")
            self.robot_s = last[0]
            self.car_s = min(self.robot_s + self.scene.case["car_ahead"], self.scene.path.length - 2.5)
            self.car = self.scene.car_at(self.car_s)
            self.car_visible = True
            self.status = "3/7 car ahead, stop"
            cancel = handle.cancel_goal_async()
            self.wait_until(cancel.done, 10.0, "取消导航")
            robot = self.wait_still(1.0)
            self.robot_s, offset = self.scene.path.project(robot[0], robot[1], self.robot_s - 2.0, self.robot_s + 2.0)
            self.log("来车", "前方来车，上位机取消导航",
                     robot=[round(v, 3) for v in robot], robot_s=round(self.robot_s, 2),
                     robot_offset_from_center=round(offset, 2),
                     car=[round(v, 3) for v in self.car], car_s=round(self.car_s, 2))
            return robot

        def check_passable(self, robot):
            self.status = "4/7 passable check"
            if not self.passable.wait_for_service(timeout_sec=20.0):
                raise Fail("没有 /check_car_passable")
            request = IsCarPassable.Request()
            request.robot_pose = stamped(self, *robot)
            request.car_pose = stamped(self, self.car[0], self.car[1], self.car[2], z=100.0)
            request.size.x, request.size.y, request.size.z = CAR_SIZE
            request.find_parking_point = False
            future = self.passable.call_async(request)
            self.wait_until(future.done, 20.0, "通过性结果")
            passable = future.result().is_car_passable
            self.log("通过性", "机器人原地挡路" if not passable else "车可以直接过去", passable=passable)
            if passable:
                raise Fail("机器人站在车道上，通过性却返回可通过")

        def avoid(self, robot):
            self.status = "5/7 avoidance"
            expect = self.scene.expect
            self.stop_points_pub.publish(PoseArray())
            self.publish_no_go()
            self.spin_for(0.5)
            if not self.avoid_client.wait_for_server(timeout_sec=30.0):
                raise Fail("没有 /garage_vehicle_avoidance")
            goal = GarageVehicleAvoidance.Goal()
            goal.polygons = self.polygons.polygons
            goal.cars_information = self.car_message()
            goal.behavior_tree = ""
            self.parking = None
            self.right_edge_path = None
            self.right_edge_wall = None
            self.parking_since = self.sim_now()
            states = []

            def feedback(message):
                state = int(message.feedback.state)
                if not states or states[-1] != state:
                    states.append(state)

            self.avoid_wall = time.time()
            self.trigger_wall["avoid"] = self.avoid_wall
            future = self.avoid_client.send_goal_async(goal, feedback_callback=feedback)
            handle = self.wait_until(future.done, 15.0, "避车动作接收") and future.result()
            if not handle.accepted:
                raise Fail("避车动作被拒绝")
            result = handle.get_result_async()
            started = time.monotonic()
            self.wait_until(lambda: self.parking or result.done() or self.right_edge_path, 90.0, "找点结果")
            if self.parking is not None:
                p = self.parking.pose
                first = (p.position.x, p.position.y, yaw_of(p.orientation))
                self.log("找点", "第一次找到停靠点", target=[round(v, 3) for v in first],
                         z=round(p.position.z, 1), where=self.scene.describe(first[0], first[1], self.robot_s))
            if expect["right_edge"]:
                self.status = "5/7 wait for right-edge path"
                self.wait_until(lambda: self.right_edge_path or result.done(), 60.0, "靠右路径 /garage_path")
                if not self.right_edge_path:
                    raise Fail(f"应该走靠右却没有靠右路径，动作状态 {result.result().status}，反馈 {states}")
                self.log("靠右", "行为树发出靠右路径",
                         seconds_after_avoid=round(self.right_edge_wall - self.avoid_wall, 1),
                         poses=len(self.right_edge_path.poses))
            self.status = "5/7 avoidance: parking"

            def target():
                if self.parking is None:
                    return None
                p = self.parking.pose
                return p.position.x, p.position.y, yaw_of(p.orientation)

            def parked():
                pose = self.robot()
                goal_pose = target()
                if result.done() or pose is None or goal_pose is None:
                    return result.done()
                if math.hypot(pose[0] - goal_pose[0], pose[1] - goal_pose[1]) > POSITION_TOLERANCE:
                    return False
                return angle_diff(pose[2], goal_pose[2]) < YAW_TOLERANCE or \
                    angle_diff(pose[2], goal_pose[2] + math.pi) < YAW_TOLERANCE

            self.wait_until(parked, 180.0, "机器人停到停靠点或避车动作结束")
            if result.done():
                wrapped = result.result()
                ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success
                self.log("避车结束", "没有停靠，避车动作已结束", status=wrapped.status,
                         reason=wrapped.result.reason, feedback_states=states,
                         right_edge=self.right_edge_path is not None)
                if expect["park"] not in ("none", "any"):
                    raise Fail(f"应停在{expect['park']}侧，机器人没停好避车动作就结束了，状态 {wrapped.status}，"
                               f"{wrapped.result.reason}，反馈 {states}")
                if not ok:
                    raise Fail(f"避车动作失败：状态 {wrapped.status}，{wrapped.result.reason}")
                self.check_timeline()
                return result, states
            if expect["park"] == "none":
                raise Fail("两侧都没有位置，机器人却停到了停靠点")
            pose = self.wait_still(1.5)
            goal_pose = target()
            if math.hypot(pose[0] - goal_pose[0], pose[1] - goal_pose[1]) > POSITION_TOLERANCE:
                raise Fail("机器人停稳后偏离停靠点")
            error = self.localization_error()
            if error is None or error > 0.5:
                raise Fail(f"停靠后定位和 Gazebo 真值差 {error} m")
            near_s, _ = self.scene.path.project(goal_pose[0], goal_pose[1], self.robot_s - 12.0, self.robot_s + 12.0)
            heading = self.scene.corridor_heading(pose[0], pose[1], near_s, 1.5)
            diff = angle_diff(pose[2], heading)
            off_tangent = math.degrees(min(diff, math.pi - diff))
            where = self.scene.describe(pose[0], pose[1], self.robot_s)
            self.log("停靠", "机器人已停在停靠点",
                     robot=[round(v, 3) for v in pose], target=[round(v, 3) for v in goal_pose],
                     distance=round(math.hypot(pose[0] - goal_pose[0], pose[1] - goal_pose[1]), 3),
                     yaw_vs_corridor_tangent_deg=round(off_tangent, 1), where=where,
                     heading="顺行驶方向" if diff < math.pi / 2.0 else "逆行驶方向",
                     seconds=round(time.monotonic() - started, 1))
            if off_tangent > MAX_OFF_TANGENT_DEG:
                raise Fail(f"停靠车头和通道切线夹角 {off_tangent:.0f}°，超过 {MAX_OFF_TANGENT_DEG}°")
            # 倒进停靠点，车走后直接接着开，不应该掉头。靠右路径本身会让机器人调头往回开，不查。
            if self.right_edge_path is None and diff >= math.pi / 2.0:
                raise Fail("停靠后车头逆着行驶方向，机器人掉头了")
            if expect["park"] == "right" and where["lateral"] > 0:
                raise Fail("应停行驶方向右侧，机器人却停在了左侧")
            if expect["park"] == "left" and where["lateral"] < 0:
                raise Fail("应停行驶方向左侧，机器人却停在了右侧")
            self.check_timeline()
            return result, states

        def check_timeline(self):
            """检查本轮时间线，保存的远侧点等待无车后直接批准。"""
            self.spin_for(0.5)
            with open(self.bt_log, errors="replace") as handle:
                bt_text = handle.read()
            with open(self.find_log, errors="replace") as handle:
                find_text = handle.read()
            events = [event for event in read_events(self.bt_log) if event[0] >= self.avoid_wall - 1.0]
            checked = evaluate_timeline(events, self.scene.expect, self.scene.dual,
                                        self.right_edge_path is not None, find_text, bt_text)
            relative = {name: round(stamp - self.avoid_wall, 2)
                        for name, stamp in checked["first"].items()}
            self.report["行为树时间线"] = [[round(stamp - self.avoid_wall, 2), name]
                                         for stamp, name, _ in events]
            self.report["批准时 side"] = checked["approve_side"]
            self.log("时间线", "行为树关键事件（相对避车开始的秒数）", first=relative,
                     approve_side=checked["approve_side"], problems=checked["problems"],
                     policy="保存远侧点，等待后直接导航")
            if checked["problems"]:
                raise Fail("；".join(checked["problems"]))

        def car_passes(self, result, states):
            self.status = "6/7 car passing"
            already_done = result.done()
            s = self.car_s
            last = time.monotonic()
            while s > -3.0:
                self.spin_for(0.05)
                now = time.monotonic()
                s -= CAR_SPEED * (now - last)
                last = now
                self.car = self.scene.car_at(s)
                if result.done() and not already_done:
                    wrapped = result.result()
                    raise Fail(f"车还没开走避车动作就结束了，状态 {wrapped.status}，{wrapped.result.reason}")
            self.car_visible = False
            self.log("车离开", "车开出通道，上位机不再发车辆")
            if already_done:
                return
            self.status = "6/7 car gone, wait 15 s"
            gone = time.monotonic()
            self.wait_until(result.done, 45.0, "避车动作结束")
            wrapped = result.result()
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not wrapped.result.success:
                raise Fail(f"避车动作失败：状态 {wrapped.status}，{wrapped.result.reason}，反馈 {states}")
            self.log("避车结束", "避车动作成功", wait_seconds=round(time.monotonic() - gone, 1),
                     feedback_states=states, reason=wrapped.result.reason)

        def resume(self):
            self.status = "7/7 resume driving"
            robot = self.robot()
            s, _ = self.scene.path.project(robot[0], robot[1], self.robot_s - 10.0, self.robot_s + 12.0)
            handle = self.send_lane(min(s + 2.5, self.scene.end_s))
            result = handle.get_result_async()
            self.wait_until(result.done, 300.0, "恢复导航到终点")
            if result.result().status != GoalStatus.STATUS_SUCCEEDED:
                raise Fail(f"恢复导航失败，状态 {result.result().status}")
            pose = self.wait_still(1.0)
            gx, gy, _ = self.scene.lane(self.scene.end_s)
            distance = math.hypot(pose[0] - gx, pose[1] - gy)
            if distance > 0.7:
                raise Fail(f"恢复导航后离终点 {distance:.2f} m")
            self.log("恢复行驶", "到达终点", robot=[round(v, 3) for v in pose], distance=round(distance, 3))

        def run(self):
            self.localize()
            self.build_corridor()
            robot = self.drive_until_car()
            self.check_passable(robot)
            result, states = self.avoid(robot)
            self.car_passes(result, states)
            self.resume()
            self.status = "PASS"
            self.report["result"] = "PASS"
            self.save()
            self.spin_for(1.0)

    scene = Scene(args.case)
    rclpy.init()
    node = UpperComputer(scene, args.report)
    print(f"用例：{scene.name}（{scene.case['title']}）", flush=True)
    code = 0
    try:
        node.run()
        print("PASS 避车全流程完成", flush=True)
    except Exception as error:
        node.status = "FAIL (see log)"
        node.report["result"] = f"FAIL {error}"
        node.report["error_type"] = type(error).__name__
        node.save()
        node.spin_for(1.0)
        print(f"FAIL {error}", flush=True)
        code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", default=DEFAULT_CASE)
    parser.add_argument("--report", default="/tmp/avoid_test_report.json")
    parser.add_argument("--print-launch-args", action="store_true",
                        help="打印 Gazebo 生成位姿、map -> odom、world、map")
    parser.add_argument("--rviz-template")
    parser.add_argument("--rviz-out")
    parser.add_argument("--check-interface", action="store_true",
                        help="检查当前加载的找点接口，不启动机器人")
    parser.add_argument("--review-log-dir",
                        help="按当前策略只重评已有日志的时间线，不改历史报告")
    args = parser.parse_args()
    if args.check_interface:
        check_interface()
        return
    if args.review_log_dir:
        raise SystemExit(review_log_dir(args.review_log_dir))
    if args.print_launch_args:
        main_prepare(args)
        return
    run_ros(args)


if __name__ == "__main__":
    main()
