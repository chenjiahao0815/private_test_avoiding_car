#!/usr/bin/env python3
"""模拟上位机走一遍避车：定位、发通道、行驶、来车、查通过性、避车动作、车走后恢复行驶。

通道形状、行驶方向、来车出现位置都在 场景用例.py 里，用 --case 选择。
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
ROBOT_LANE = 0.95          # 机器人离右侧长边的距离
CAR_LANE = 0.45            # 来车偏离中心线、靠左侧（对机器人而言）
CAR_SIZE = (4.5, 1.8, 1.5)
CAR_SPEED = 1.5
START_S = 1.5
END_MARGIN = 1.5
GOAL_SPACING = 2.0
POSITION_TOLERANCE = 0.35
YAW_TOLERANCE = 0.2
# 停靠后车头大体沿通道切线（不分顺逆），超过这个角度判失败。
MAX_OFF_TANGENT_DEG = 35
DEFAULT_CASE = "原地图弧形_左到右"


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class Scene:
    """一个用例的几何：通道多边形、机器人车道、来车车道。"""

    def __init__(self, name):
        if name not in cases.CASES:
            raise SystemExit(f"没有用例 {name}，可选：{', '.join(cases.CASES)}")
        self.case = cases.CASES[name]
        self.name = name
        self.path = self.case["path"]
        self.polygon = self.case["polygon"]
        self.world = cases.WORLDS[self.case["world"]]
        self.end_s = self.path.length - END_MARGIN

    def lane(self, s):
        return self.path.offset(s, -(HALF_WIDTH - ROBOT_LANE))

    def car_at(self, s):
        x, y, heading = self.path.offset(s, CAR_LANE)
        return x, y, heading + math.pi

    @staticmethod
    def external_s(robot_s):
        return robot_s - 3.5

    def external_point(self, robot_s):
        """通道外右侧的停靠点，车头沿通道切线、顺着机器人行驶方向。"""
        return self.path.offset(self.external_s(robot_s), -(HALF_WIDTH + 1.3))

    def corridor_heading(self, x, y, near_s, window):
        # 直角内拐角处离两条边一样近，窗口要小，否则会投到另一条边上。
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
        return min(xs), min(ys), max(xs), max(ys)

    def describe(self, x, y, robot_s):
        """停靠点相对通道的位置：在机器人前/后多少米，在行驶方向左/右多少米，是否在通道里。"""
        s, _ = self.path.project(x, y, robot_s - 10.0, robot_s + 10.0)
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
                self.world["world"] or "-", self.world["map"] or "-"]


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
    from garage_road_model.msg import RoadModelArray
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
        def __init__(self, scene, external, external_no_yaw, report_path):
            super().__init__("avoid_test_upper", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.scene = scene
            self.external = external or external_no_yaw
            self.external_no_yaw = external_no_yaw
            self.report_path = report_path
            mode = "external_no_yaw" if external_no_yaw else "external" if external else "auto"
            self.report = {"case": scene.name, "title": scene.case["title"], "mode": mode}
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.polygon_pub = self.create_publisher(Polygons, "/garage_polygons", latched)
            self.stop_points_pub = self.create_publisher(PoseArray, "/vehicle_avoidance_stop_points", latched)
            self.car_pub = self.create_publisher(CarDetectArray, "/car_information", 10)
            self.marker_pub = self.create_publisher(MarkerArray, "/avoid_test/markers", latched)
            self.road = None
            self.parking = None
            self.right_edge_path = None
            self.create_subscription(RoadModelArray, "/garage_road_models", self.on_road, latched)
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
            self.external_target = None
            self.parking_since = None
            self.status = "starting"
            self.create_timer(0.1, self.publish_car)
            self.create_timer(0.5, self.publish_markers)

        # ---------- 通用 ----------
        def on_road(self, msg):
            self.road = msg

        def on_parking(self, msg):
            if self.parking_since is None:
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if stamp + 1e-3 >= self.parking_since:
                self.parking = msg

        def on_right_edge(self, msg):
            if msg.poses:
                self.right_edge_path = msg

        def log(self, step, text, **data):
            line = f"[{step}] {text}"
            if data:
                line += "  " + json.dumps(data, ensure_ascii=False)
            print(line, flush=True)
            if data:
                self.report[step] = data
            self.save()

        def save(self):
            with open(self.report_path, "w") as handle:
                json.dump(self.report, handle, ensure_ascii=False, indent=2)

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

        def publish_car(self):
            self.car_pub.publish(self.car_message())

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

            stop_point = base("external_stop_point", 5, Marker.ARROW)
            if self.external_target:
                x, y, yaw = self.external_target
                stop_point.pose = stamped(self, x, y, yaw or 0.0).pose
                stop_point.scale.x, stop_point.scale.y, stop_point.scale.z = 1.0, 0.2, 0.2
                stop_point.color.r, stop_point.color.g, stop_point.color.a = 1.0, 0.5, 1.0
            else:
                stop_point.action = Marker.DELETE
            markers.markers.append(stop_point)

            x0, y0, x1, y1 = self.scene.bounds()
            status = base("status", 4, Marker.TEXT_VIEW_FACING)
            status.pose.position.x = (x0 + x1) / 2.0
            status.pose.position.y = y1 + 2.0
            status.pose.position.z = 1.0
            status.scale.z = 0.9
            status.color.b, status.color.a = 0.6, 1.0
            mode = "external point, no yaw" if self.external_no_yaw else \
                "external point" if self.external else "self search"
            status.text = f"{self.scene.case['label']} ({mode})   {self.status}"
            markers.markers.append(status)
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
            self.status = "2/7 corridor fitting"
            polygon = Polygon()
            polygon.points = [Point32(x=float(x), y=float(y)) for x, y in self.scene.polygon]
            self.polygons = Polygons()
            self.polygons.polygons.append(polygon)
            self.road = None
            self.polygon_pub.publish(self.polygons)

            def road_ready():
                if not self.road or not self.road.roads:
                    return None
                return self.road.roads[0]

            road = self.wait_until(road_ready, 30.0, "道路模型 /garage_road_models")
            # 拟合失败时节点照样发布，两侧采样为空。
            self.spin_for(1.0)
            road = self.road.roads[0]
            samples = [len(road.side_a.samples), len(road.side_b.samples)]
            if not (samples[0] and samples[1]):
                self.log("拟合", "通道拟合失败", vertices=len(polygon.points), samples=samples)
                raise Fail("通道拟合失败：两侧长边没有采样，详见 道路拟合 日志")
            self.log("拟合", "通道拟合成功", vertices=len(polygon.points), samples=samples,
                     lengths=[round(road.side_a.length, 2), round(road.side_b.length, 2)])

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
            self.log("行驶", "沿通道右侧车道向前开", trigger_s=trigger, length=round(self.scene.path.length, 1))
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
            self.status = "5/7 avoidance: find point"
            stop_points = PoseArray()
            stop_points.header.frame_id = "map"
            expected_external = None
            external_inside = False
            if self.external:
                expected_external = self.scene.external_point(self.robot_s)
                external_inside = self.scene.inside(expected_external[0], expected_external[1])
                if self.external_no_yaw:
                    # 不带朝向（单位四元数），由找点节点自己定车头；这时只核对车头是否沿通道切线。
                    stop_points.poses.append(stamped(self, expected_external[0], expected_external[1], 0.0).pose)
                    expected_external = (expected_external[0], expected_external[1], None)
                else:
                    stop_points.poses.append(stamped(self, *expected_external).pose)
                self.external_target = expected_external
            self.stop_points_pub.publish(stop_points)
            self.spin_for(0.5)
            if not self.avoid_client.wait_for_server(timeout_sec=30.0):
                raise Fail("没有 /garage_vehicle_avoidance")
            goal = GarageVehicleAvoidance.Goal()
            goal.polygons = self.polygons.polygons
            goal.cars_information = self.car_message()
            goal.behavior_tree = ""
            self.parking = None
            self.right_edge_path = None
            self.parking_since = self.sim_now()
            states = []

            def feedback(message):
                state = int(message.feedback.state)
                if not states or states[-1] != state:
                    states.append(state)

            future = self.avoid_client.send_goal_async(goal, feedback_callback=feedback)
            handle = self.wait_until(future.done, 15.0, "避车动作接收") and future.result()
            if not handle.accepted:
                raise Fail("避车动作被拒绝")
            result = handle.get_result_async()
            started = time.monotonic()
            target = None
            if self.external:
                # 找点节点只给自搜索点发 /marker_parking_point，外部点没有标记，目标就是下发的点。
                target = expected_external
                self.log("找点", "上位机下发外部停靠点" + ("（不带朝向）" if self.external_no_yaw else ""),
                         target=[None if v is None else round(v, 3) for v in target],
                         external_in_corridor=external_inside,
                         where=self.scene.describe(target[0], target[1], self.robot_s))
            else:
                self.wait_until(lambda: self.parking or result.done() or self.right_edge_path, 90.0, "找点结果")
                if self.parking is None:
                    branch = "右边沿路径" if self.right_edge_path else "无"
                    status = result.result().status if result.done() else "运行中"
                    raise Fail(f"行为树没有找到停靠点（走了 {branch} 分支，动作状态 {status}，反馈 {states}）")
                p = self.parking.pose
                target = (p.position.x, p.position.y, yaw_of(p.orientation))
                self.log("找点", "行为树自搜索到停靠点", target=[round(v, 3) for v in target],
                         z=round(p.position.z, 1), where=self.scene.describe(target[0], target[1], self.robot_s))
            self.status = "5/7 avoidance: parking"
            fallback = []

            def parked():
                nonlocal target
                pose = self.robot()
                if self.external and self.parking and not fallback:
                    p = self.parking.pose
                    fallback.append((p.position.x, p.position.y, yaw_of(p.orientation)))
                    if not external_inside:
                        raise Fail("外部点在通道外，行为树却用了自搜索点")
                    target = fallback[0]
                    self.log("找点", "外部点在通道内，行为树改用自搜索点", target=[round(v, 3) for v in target],
                             z=round(p.position.z, 1), where=self.scene.describe(target[0], target[1], self.robot_s))
                if result.done() or pose is None:
                    return result.done()
                if math.hypot(pose[0] - target[0], pose[1] - target[1]) > POSITION_TOLERANCE:
                    return False
                if target[2] is None:
                    return True
                return angle_diff(pose[2], target[2]) < YAW_TOLERANCE or \
                    angle_diff(pose[2], target[2] + math.pi) < YAW_TOLERANCE

            self.wait_until(parked, 150.0, "机器人停到停靠点")
            if result.done():
                wrapped = result.result()
                raise Fail(f"机器人还没停好避车动作就结束了，状态 {wrapped.status}，"
                           f"{wrapped.result.reason}，反馈 {states}")
            pose = self.wait_still(1.5)
            if math.hypot(pose[0] - target[0], pose[1] - target[1]) > POSITION_TOLERANCE:
                raise Fail("机器人停稳后偏离停靠点")
            if target[2] is None:
                yaw_vs_target = "上位机未给朝向"
            else:
                yaw_error = angle_diff(pose[2], target[2])
                yaw_vs_target = "一致" if yaw_error < YAW_TOLERANCE else \
                    "相差 180 度" if angle_diff(pose[2], target[2] + math.pi) < YAW_TOLERANCE else \
                    f"相差 {math.degrees(yaw_error):.0f} 度"
            error = self.localization_error()
            if error is None or error > 0.5:
                raise Fail(f"停靠后定位和 Gazebo 真值差 {error} m")
            if self.external and target is expected_external:
                # 外部点可能在直角内拐角，偏移量比行驶线圆角半径还大，投影不回原弧长；直接用下发处的切线。
                heading = self.scene.path.at(self.scene.external_s(self.robot_s))[2]
            else:
                near_s, _ = self.scene.path.project(target[0], target[1], self.robot_s - 10.0, self.robot_s + 10.0)
                heading = self.scene.corridor_heading(pose[0], pose[1], near_s, 1.5)
            diff = angle_diff(pose[2], heading)
            off_tangent = math.degrees(min(diff, math.pi - diff))
            self.log("停靠", "机器人已停在停靠点",
                     robot=[round(v, 3) for v in pose],
                     distance=round(math.hypot(pose[0] - target[0], pose[1] - target[1]), 3),
                     yaw_vs_target=yaw_vs_target,
                     yaw_vs_corridor_tangent_deg=round(off_tangent, 1),
                     heading="顺行驶方向" if diff < math.pi / 2.0 else "逆行驶方向",
                     where=self.scene.describe(pose[0], pose[1], self.robot_s),
                     seconds=round(time.monotonic() - started, 1))
            if off_tangent > MAX_OFF_TANGENT_DEG:
                raise Fail(f"停靠车头和通道切线夹角 {off_tangent:.0f}°，超过 {MAX_OFF_TANGENT_DEG}°")
            return result, states

        def car_passes(self, result, states):
            self.status = "6/7 car passing"
            s = self.car_s
            last = time.monotonic()
            while s > -3.0:
                self.spin_for(0.05)
                now = time.monotonic()
                s -= CAR_SPEED * (now - last)
                last = now
                self.car = self.scene.car_at(s)
                if result.done():
                    wrapped = result.result()
                    raise Fail(f"车还没开走避车动作就结束了，状态 {wrapped.status}，{wrapped.result.reason}")
            self.car_visible = False
            self.status = "6/7 car gone, wait 15 s"
            self.log("车离开", "车开出通道，上位机不再发车辆")
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
            s, _ = self.scene.path.project(robot[0], robot[1], self.robot_s - 8.0, self.robot_s + 3.0)
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
    node = UpperComputer(scene, args.external, args.external_no_yaw, args.report)
    mode = "外部停靠点（不带朝向）" if args.external_no_yaw else "外部停靠点" if args.external else "自搜索停靠点"
    print(f"用例：{scene.name}（{scene.case['title']}），模式：{mode}", flush=True)
    code = 0
    try:
        node.run()
        print("PASS 避车全流程完成", flush=True)
    except Fail as error:
        node.status = "FAIL (see log)"
        node.report["result"] = f"FAIL {error}"
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
    parser.add_argument("--external", action="store_true", help="用上位机下发的外部停靠点")
    parser.add_argument("--external-no-yaw", action="store_true", help="下发不带朝向的外部停靠点")
    parser.add_argument("--report", default="/tmp/avoid_test_report.json")
    parser.add_argument("--print-launch-args", action="store_true",
                        help="打印 Gazebo 生成位姿、map -> odom、world、map")
    parser.add_argument("--rviz-template")
    parser.add_argument("--rviz-out")
    args, _ = parser.parse_known_args()
    if args.print_launch_args:
        main_prepare(args)
        return
    run_ros(args)


if __name__ == "__main__":
    main()
