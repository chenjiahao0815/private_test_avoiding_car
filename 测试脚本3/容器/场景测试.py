#!/usr/bin/env python3
"""单场景集成测试：实体障碍进入代价地图 -> 自搜索停车 -> 来车离开 -> 恢复导航。"""
import argparse
import json
import math
from pathlib import Path
import time
import traceback

from 生成场景 import CORRIDOR, FINISH, OBSTACLES, SLOT, START, TRIGGER_X


def yaw_of(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def yaw_error(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def overlaps(polygon, rect):
    """分离轴判定真实 footprint 与矩形是否相交，不把机器人简化成中心点。"""
    ax, ay, bx, by = rect
    other = [(ax, ay), (bx, ay), (bx, by), (ax, by)]
    axes = [(1, 0), (0, 1)]
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        axes.append((b[1]-a[1], a[0]-b[0]))
    for dx, dy in axes:
        p = [x*dx + y*dy for x, y in polygon]
        q = [x*dx + y*dy for x, y in other]
        if max(p) <= min(q) or max(q) <= min(p):
            return False
    return True


def check_interface():
    from ament_index_python.packages import get_package_prefix
    from capella_ros_msg.action import FindCarAvoidancePoint
    from rclpy.serialization import deserialize_message, serialize_message
    goal = FindCarAvoidancePoint.Goal()
    goal.enable_far_side = True
    assert deserialize_message(serialize_message(goal), type(goal)).enable_far_side
    assert hasattr(FindCarAvoidancePoint.Result(), "side")
    for name in ("capella_ros_msg", "garage_behavior_tree", "garage_navigator", "find_free_space"):
        prefix = get_package_prefix(name)
        assert prefix.startswith("/root/avoid_scene3_ws/install/"), (name, prefix)
        print(f"{name}: {prefix}", flush=True)
    xml = Path(get_package_prefix("garage_navigator")) / "share/garage_navigator/behavior_trees/garage_vehicle_avoidance_dual_side.xml"
    assert 'StopWhenCarsGone idle_timeout="3.0"' in xml.read_text()
    print("接口序列化及当前双侧行为树检查通过", flush=True)


def run(report_path):
    import rclpy
    from action_msgs.msg import GoalStatus
    from capella_ros_msg.msg import CarDetectArray, CarDetectSingle
    from capella_ros_msg.srv import IsCarPassable
    from garage_utils_msgs.action import GarageVehicleAvoidance
    from garage_utils_msgs.msg import Polygons
    from gazebo_msgs.srv import GetEntityState
    from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped, PoseArray, PoseStamped
    from nav2_msgs.action import NavigateThroughPoses
    from nav2_msgs.msg import Costmap
    from nav_msgs.msg import Path as NavPath
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker, MarkerArray

    class Tester(Node):
        def __init__(self):
            super().__init__("avoid_scene3_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
            self.report_path = Path(report_path)
            self.report = {"result": "RUNNING", "scenario": "上侧双障碍之间停靠", "events": [],
                           "no_go_polygons": 0, "external_parking_points": 0, "collision_samples": []}
            self.trace = []
            self.phase = "starting"
            self.car = None
            self.car_speed = 0.0
            self.parking = None
            self.search_start = None
            self.right_edge = False
            self.costmaps = {}
            self.footprint = None
            self.handles = []
            latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.polygon_pub = self.create_publisher(Polygons, "/garage_polygons", latched)
            self.no_go_pub = self.create_publisher(Polygons, "/cleaning_tool_retraction_areas", latched)
            self.points_pub = self.create_publisher(PoseArray, "/vehicle_avoidance_stop_points", latched)
            self.car_pub = self.create_publisher(CarDetectArray, "/car_information", 10)
            self.marker_pub = self.create_publisher(MarkerArray, "/avoid_test/markers", latched)
            self.create_subscription(Marker, "/marker_parking_point", self.on_parking, latched)
            self.create_subscription(NavPath, "/garage_path", self.on_right_edge, 10)
            self.create_subscription(PolygonStamped, "/global_costmap/published_footprint", self.on_footprint, 10)
            for name in ("global", "local"):
                self.create_subscription(Costmap, f"/{name}_costmap/costmap_raw",
                                         lambda msg, name=name: self.costmaps.__setitem__(name, msg), latched)
            self.buffer = Buffer()
            self.listener = TransformListener(self.buffer, self)
            self.nav = ActionClient(self, NavigateThroughPoses, "/navigate_through_poses")
            self.avoid = ActionClient(self, GarageVehicleAvoidance, "/garage_vehicle_avoidance")
            self.passable = self.create_client(IsCarPassable, "/check_car_passable")
            self.truth = self.create_client(GetEntityState, "/get_entity_state")
            self.create_timer(0.1, self.tick)
            self.create_timer(0.5, self.markers)

        def stamp(self, x, y, yaw=math.pi, z=0.0):
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = float(x), float(y), float(z)
            msg.pose.orientation.z, msg.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            return msg

        def now(self):
            return self.get_clock().now().nanoseconds * 1e-9

        def robot(self):
            try:
                tf = self.buffer.lookup_transform("map", "base_link", rclpy.time.Time()).transform
                return tf.translation.x, tf.translation.y, yaw_of(tf.rotation)
            except Exception:
                return None

        def on_parking(self, msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            if self.search_start is not None and stamp >= self.search_start - 0.001:
                self.parking = msg.pose.position.x, msg.pose.position.y, yaw_of(msg.pose.orientation)

        def on_right_edge(self, msg):
            if self.search_start is not None and msg.poses:
                self.right_edge = True

        def on_footprint(self, msg):
            if msg.header.frame_id != "map" or not msg.polygon.points:
                return
            self.footprint = [(p.x, p.y) for p in msg.polygon.points]
            for name, ax, ay, bx, by, _ in OBSTACLES:
                if overlaps(self.footprint, (ax, ay, bx, by)):
                    if len(self.report["collision_samples"]) < 20:
                        self.report["collision_samples"].append({"time": self.now(), "obstacle": name,
                                                                 "footprint": self.footprint})

        def cars(self):
            msg = CarDetectArray()
            if self.car is not None:
                single = CarDetectSingle()
                single.pose = self.stamp(*self.car, z=100.0)
                single.size.x, single.size.y, single.size.z = 4.5, 1.8, 1.5
                single.speed.header.frame_id = "map"
                single.speed.twist.linear.x = self.car_speed
                msg.results.append(single)
            return msg

        def tick(self):
            # 模拟实际发布端：有动车才发布，离开后停止消息，而非依赖空数组心跳。
            if self.car is not None:
                self.car_pub.publish(self.cars())
            pose = self.robot()
            if pose:
                self.trace.append([round(self.now(), 3), *pose, self.phase])

        def markers(self):
            array = MarkerArray()
            outline = Marker()
            outline.header = self.stamp(0, 0).header
            outline.ns, outline.id, outline.type = "corridor", 0, Marker.LINE_STRIP
            outline.pose.orientation.w = 1.0
            outline.scale.x = 0.04
            outline.color.r, outline.color.g, outline.color.a = 0.9, 0.7, 1.0
            outline.points = [Point(x=x, y=y, z=0.04) for x, y in CORRIDOR + CORRIDOR[:1]]
            array.markers.append(outline)
            car = Marker()
            car.header = outline.header
            car.ns, car.id, car.type = "car", 1, Marker.CUBE
            if self.car:
                car.pose = self.stamp(*self.car, z=0.75).pose
                car.scale.x, car.scale.y, car.scale.z = 4.5, 1.8, 1.5
                car.color.r, car.color.a = 0.9, 0.8
            else:
                car.action = Marker.DELETE
            array.markers.append(car)
            self.marker_pub.publish(array)

        def save(self):
            self.report_path.write_text(json.dumps(self.report, ensure_ascii=False, indent=2))

        def log(self, step, **data):
            self.phase = step
            self.report["events"].append({"step": step, "sim_time": round(self.now(), 3), **data})
            self.save()
            print(f"[{step}] {json.dumps(data, ensure_ascii=False)}", flush=True)

        def wait(self, predicate, timeout, label):
            until = time.monotonic() + timeout
            while time.monotonic() < until:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.report["collision_samples"]:
                    raise RuntimeError(f"机器人 footprint 进入实体障碍：{self.report['collision_samples'][-1]}")
                value = predicate()
                if value:
                    return value
            raise TimeoutError(f"等待 {label} 超时（{timeout}s）")

        def settle(self, seconds=1.0):
            last, since = None, time.monotonic()
            def still():
                nonlocal last, since
                pose = self.robot()
                if pose is None:
                    return False
                if last is None or math.dist(last[:2], pose[:2]) > 0.01 or yaw_error(last[2], pose[2]) > 0.01:
                    last, since = pose, time.monotonic()
                return pose if time.monotonic() - since >= seconds else False
            return self.wait(still, 30, "机器人停稳")

        def cost(self, name, x, y):
            msg = self.costmaps.get(name)
            if msg is None:
                return None
            meta = msg.metadata
            ix, iy = int((x-meta.origin.position.x)/meta.resolution), int((y-meta.origin.position.y)/meta.resolution)
            if not (0 <= ix < meta.size_x and 0 <= iy < meta.size_y):
                return None
            return int(msg.data[iy*meta.size_x+ix])

        def check_maps(self):
            samples = {name: ((ax+bx)/2, (ay+by)/2) for name, ax, ay, bx, by, _ in OBSTACLES}
            self.wait(lambda: all(self.cost("global", *p) == 254 for p in samples.values()), 30, "全局障碍栅格=254")
            self.wait(lambda: self.cost("local", 28.0, 22.0) == 254, 30, "局部上侧障碍栅格=254")
            assert self.cost("global", 33.0, 22.8) < 253, "车位中间被障碍封住"
            self.log("代价地图确认", global_obstacles={k: self.cost("global", *v) for k, v in samples.items()},
                     local_obstacle=self.cost("local", 28.0, 22.0), slot_center=self.cost("global", 33.0, 22.8))

        def send_nav(self, start_x):
            assert self.nav.wait_for_server(timeout_sec=30), "导航 Action 不存在"
            goal = NavigateThroughPoses.Goal()
            x = start_x
            while x > FINISH[0]+0.5:
                goal.poses.append(self.stamp(x, FINISH[1]))
                x -= 2.0
            goal.poses.append(self.stamp(*FINISH))
            future = self.nav.send_goal_async(goal)
            self.wait(future.done, 15, "导航接收")
            handle = future.result()
            assert handle.accepted, "导航被拒绝"
            self.handles.append(handle)
            return handle, handle.get_result_async()

        def check_truth(self):
            assert self.truth.wait_for_service(timeout_sec=10), "Gazebo 真值服务不存在"
            req = GetEntityState.Request()
            req.name, req.reference_frame = "turtlebot3_waffle", "world"
            future = self.truth.call_async(req)
            self.wait(future.done, 10, "Gazebo 真值")
            result = future.result()
            assert result.success, "Gazebo 没有机器人模型"
            p = result.state.pose.position
            error = math.dist(self.robot()[:2], (p.x, p.y))
            assert error < 0.15, f"定位与 Gazebo 偏差 {error}m"
            return round(error, 4)

        def execute(self):
            self.wait(lambda: self.robot() and len(self.costmaps) == 2 and self.footprint, 90, "TF、代价地图和 footprint")
            pose = self.settle()
            assert math.dist(pose[:2], START[:2]) < 0.3, f"起点错误：{pose}"
            self.log("定位", pose=pose, gazebo_error=self.check_truth())
            polygons = Polygons()
            poly = Polygon()
            poly.points = [Point32(x=x, y=y) for x, y in CORRIDOR]
            polygons.polygons.append(poly)
            self.polygon_pub.publish(polygons)
            self.no_go_pub.publish(Polygons())
            self.points_pub.publish(PoseArray())
            self.log("行驶", direction="向左", no_go_polygons=0, external_points=0)
            nav, nav_result = self.send_nav(START[0]-2.0)
            self.wait(lambda: nav_result.done() or self.robot()[0] <= TRIGGER_X, 240, "遇车位置")
            assert not nav_result.done(), f"遇车前导航已经结束：{nav_result.result().status}"
            self.car = (self.robot()[0]-6.5, 20.45, 0.0)
            self.car_speed = 0.5
            cancel = nav.cancel_goal_async()
            self.wait(cancel.done, 10, "取消正常导航")
            assert cancel.result().goals_canceling, "正常导航没有接受取消"
            robot = self.settle()
            self.log("来车", robot=robot, car=self.car)
            self.check_maps()
            assert self.passable.wait_for_service(timeout_sec=20), "通过性服务不存在"
            req = IsCarPassable.Request()
            req.robot_pose, req.car_pose = self.stamp(*robot), self.stamp(*self.car, z=100)
            req.size.x, req.size.y, req.size.z = 4.5, 1.8, 1.5
            req.find_parking_point = False
            future = self.passable.call_async(req)
            self.wait(future.done, 20, "通过性")
            assert not future.result().is_car_passable, "原地应不可通过"
            self.log("通过性", passable=False)
            assert self.avoid.wait_for_server(timeout_sec=30), "避车 Action 不存在"
            goal = GarageVehicleAvoidance.Goal()
            goal.polygons, goal.cars_information = polygons.polygons, self.cars()
            self.search_start = self.now()
            future = self.avoid.send_goal_async(goal)
            self.wait(future.done, 15, "避车接收")
            handle = future.result()
            assert handle.accepted, "避车被拒绝"
            self.handles.append(handle)
            result = handle.get_result_async()
            self.wait(lambda: self.parking or result.done(), 90, "自搜索车位")
            assert self.parking, f"避车提前结束：{result.result()}"
            gx, gy, _ = self.parking
            assert SLOT[0] < gx < SLOT[2] and SLOT[1] < gy < SLOT[3], f"选中的点不在两障碍之间：{self.parking}"
            self.log("找点", target=self.parking, right_edge=self.right_edge)
            self.wait(lambda: result.done() or math.dist(self.robot()[:2], self.parking[:2]) < 0.35, 180, "停入车位")
            assert not result.done(), f"停靠前避车已结束：{result.result()}"
            pose = self.settle(1.5)
            distance = math.dist(pose[:2], self.parking[:2])
            assert distance < 0.35, f"停稳后偏离目标 {distance}m"
            assert yaw_error(pose[2], math.pi) < math.radians(35), f"停车方向错误：{pose[2]}"
            assert self.footprint and all(SLOT[0] < x < SLOT[2] and SLOT[1] < y < SLOT[3] for x, y in self.footprint), "车身未完全停进空位"
            self.log("停靠", robot=pose, target=self.parking, distance=distance, gazebo_error=self.check_truth())
            self.phase = "来车经过"
            previous = self.now()
            self.car_speed = 1.5
            while self.car[0] < 48.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                now = self.now()
                self.car = (self.car[0] + self.car_speed*(now-previous), self.car[1], self.car[2])
                previous = now
                assert not result.done(), "动车还存在时避车提前结束"
            self.car = None
            gone = self.now()
            self.log("车离开", publish_car_information=False)
            self.wait(result.done, 75, "停车后的独立无车等待")
            wrapped = result.result()
            assert wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success, f"避车失败：{wrapped}"
            elapsed = self.now()-gone
            assert elapsed >= 14.5, f"停车后未等待完整15秒：{elapsed}"
            self.log("避车完成", status=wrapped.status, success=wrapped.result.success, wait_sim_seconds=elapsed)
            self.phase = "恢复行驶"
            _, result = self.send_nav(min(self.robot()[0]-2.5, 41.5))
            self.wait(result.done, 300, "恢复导航")
            assert result.result().status == GoalStatus.STATUS_SUCCEEDED, f"恢复导航失败：{result.result().status}"
            pose = self.settle()
            distance = math.dist(pose[:2], FINISH[:2])
            assert distance < 0.7, f"终点偏差 {distance}m"
            assert not self.report["collision_samples"], "途中与实体障碍相交"
            self.log("恢复行驶完成", robot=pose, distance=distance, gazebo_error=self.check_truth())
            self.report["result"] = "PASS"
            self.report["right_edge_used"] = self.right_edge
            self.save()

        def artifacts(self):
            (self.report_path.parent / "trajectory.json").write_text(json.dumps(self.trace, ensure_ascii=False))
            for name, msg in self.costmaps.items():
                w, h = msg.metadata.size_x, msg.metadata.size_y
                pixels = bytearray()
                for row in range(h-1, -1, -1):
                    pixels.extend(205 if v == 255 else round(254*(1-v/254)) for v in msg.data[row*w:(row+1)*w])
                (self.report_path.parent / f"{name}_costmap.pgm").write_bytes(f"P5\n{w} {h}\n255\n".encode()+pixels)

    rclpy.init()
    node = Tester()
    code = 0
    try:
        node.execute()
        print("PASS：实体障碍、代价地图、找点停靠、车辆离开、恢复导航全部通过", flush=True)
    except Exception as exc:
        node.report["result"] = f"FAIL {exc}"
        node.report["error_type"] = type(exc).__name__
        node.save()
        traceback.print_exc()
        code = 1
    finally:
        node.artifacts()
        for handle in node.handles:
            try:
                future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, future, timeout_sec=1)
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()
    return code


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--check-interface", action="store_true")
    parser.add_argument("--report")
    args = parser.parse_args()
    if args.check_interface:
        check_interface()
    else:
        raise SystemExit(run(args.report))
