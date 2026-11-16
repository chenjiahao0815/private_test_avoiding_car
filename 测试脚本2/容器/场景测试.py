#!/usr/bin/env python3
"""mk38 原地图集成测试：实际靠右 -> 挂起找点 -> 无车 3 秒取消 -> 恢复原任务。"""
import argparse
import json
import math
from pathlib import Path
import time
import traceback

import os
G = json.loads(Path(os.environ['SCENE_GEOMETRY']).read_text())
CORRIDOR, START, FINISH = G['corridor'], G['start'], G['finish']
OBSTACLES = []


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
        assert prefix.startswith("/root/avoid_scene2_ws/install/"), (name, prefix)
        print(f"{name}: {prefix}", flush=True)
    xml = Path(get_package_prefix("garage_navigator")) / "share/garage_navigator/behavior_trees/garage_vehicle_avoidance_dual_side.xml"
    assert 'StopWhenCarsGone idle_timeout="3.0"' in xml.read_text()
    print("接口序列化及当前双侧行为树检查通过", flush=True)


def run(report_path):
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from capella_ros_msg.msg import CarDetectArray, CarDetectSingle
    from capella_ros_msg.srv import IsCarPassable
    from garage_utils_msgs.action import GarageVehicleAvoidance
    from garage_utils_msgs.msg import Polygons
    from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped, PoseArray, PoseStamped, Twist, PoseWithCovarianceStamped
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
            super().__init__("avoid_scene2_test", parameter_overrides=[Parameter("use_sim_time", value=False)])
            self.report_path = Path(report_path)
            self.report = {"result": "RUNNING", "scenario": "mk38 原地图：靠右无车退出", "events": [],
                           "no_go_polygons": 0, "external_parking_points": 0, "collision_samples": []}
            self.trace = []
            self.commands = []
            self.create_subscription(Twist, '/cmd_vel',
                lambda m: self.commands.append((self.now(), m.linear.x, m.angular.z)), 100)
            self.phase = "starting"
            self.car = None
            self.car_speed = 0.0
            self.empty_messages = False
            self.last_car_stamp = None
            self.find_events = []
            self.follow_states = {}
            self.create_subscription(GoalStatusArray, '/follow_path/_action/status',
                lambda m: self.follow_states.update({tuple(s.goal_info.goal_id.uuid): s.status for s in m.status_list}), 10)
            from std_msgs.msg import String
            self.create_subscription(String, "/test2/find_events", lambda m: self.find_events.append(json.loads(m.data)), 100)
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
                self.last_car_stamp = self.now()
            elif self.empty_messages:
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

        def confirm_stop_command(self):
            self.wait(lambda: self.commands and abs(self.commands[-1][1]) < 1e-6 and
                      abs(self.commands[-1][2]) < 1e-6, 8, '实际 cmd_vel 为零')
            start = self.now()
            count = len(self.commands)
            self.wait(lambda: self.now()-start >= .5, 5, '停车指令保持')
            assert all(abs(v)<1e-6 and abs(w)<1e-6 for _,v,w in self.commands[count:]), '取消后仍有非零速度指令'
            return self.robot()

        def cost(self, name, x, y):
            msg = self.costmaps.get(name)
            if msg is None:
                return None
            meta = msg.metadata
            ix, iy = int((x-meta.origin.position.x)/meta.resolution), int((y-meta.origin.position.y)/meta.resolution)
            if not (0 <= ix < meta.size_x and 0 <= iy < meta.size_y):
                return None
            return int(msg.data[iy*meta.size_x+ix])

        def send_nav(self, target):
            assert self.nav.wait_for_server(timeout_sec=30)
            goal = NavigateThroughPoses.Goal()
            goal.poses = [self.stamp(*target)]
            future = self.nav.send_goal_async(goal)
            self.wait(future.done, 15, '导航接收')
            handle = future.result()
            assert handle.accepted
            self.handles.append(handle)
            return handle, handle.get_result_async()

        def execute(self):
            from std_srvs.srv import SetBool
            from ament_index_python.packages import get_package_share_directory
            self.wait(lambda: self.robot() and self.footprint, 90, '定位和 footprint')
            initial_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            self.wait(lambda: initial_pub.get_subscription_count() > 0, 10, '虚拟定位初始化接口')
            initial = PoseWithCovarianceStamped()
            initial.header = self.stamp(*START).header
            initial.pose.pose = self.stamp(*START).pose
            initial.pose.covariance[0] = initial.pose.covariance[7] = initial.pose.covariance[35] = 0.01
            initial_pub.publish(initial)
            self.wait(lambda: math.dist(self.robot()[:2], START[:2]) < .15, 10, '定位到原图起点')
            pose = self.settle()
            self.log('定位检查', pose=pose, simulator='cmd_vel 运动学积分')
            control = self.create_client(SetBool, '/test2/hold_find')
            assert control.wait_for_service(timeout_sec=30)
            polygons = Polygons(polygons=[Polygon(points=[Point32(x=float(x), y=float(y)) for x,y in CORRIDOR])])
            self.polygon_pub.publish(polygons)
            self.no_go_pub.publish(Polygons())
            self.points_pub.publish(PoseArray())
            tree_dir = Path(get_package_share_directory('garage_navigator')) / 'behavior_trees'
            cases = [(tree, mode) for tree in ('garage_vehicle_avoidance.xml', 'garage_vehicle_avoidance_dual_side.xml')
                     for mode in ('empty_array', 'stop_publish')]
            for index, (tree, mode) in enumerate(cases):
                a, b = (START, FINISH) if index % 2 == 0 else (FINISH, START)
                yaw = math.atan2(b[1]-a[1], b[0]-a[0])
                target = (b[0], b[1], yaw)
                self.car = None
                self.empty_messages = False
                self.right_edge = False
                self.search_start = None
                self.find_events.clear()
                pending = control.call_async(SetBool.Request(data=False))
                self.wait(pending.done, 10, '重置找点测试服务')
                self.log('开始用例', tree=tree, disappearance=mode)
                start_pose = self.robot()
                nav, nav_result = self.send_nav(target)
                self.wait(lambda: nav_result.done() or math.dist(start_pose[:2],self.robot()[:2]) > 8.0, 90, '原任务开始行驶')
                assert not nav_result.done(), '原任务过早结束'
                cancel = nav.cancel_goal_async()
                self.wait(cancel.done, 10, '暂停原导航')
                self.wait(nav_result.done, 10, '原导航已取消')
                pose = self.settle()
                self.car = (pose[0]+6*math.cos(yaw), pose[1]+6*math.sin(yaw), yaw+math.pi)
                self.car_speed = 0.5
                self.wait(lambda: self.last_car_stamp is not None, 3, '动车发布')
                assert self.avoid.wait_for_server(timeout_sec=20)
                goal = GarageVehicleAvoidance.Goal(polygons=polygons.polygons, cars_information=self.cars(),
                                                 behavior_tree=str(tree_dir/tree))
                self.search_start = self.now()
                sent = self.avoid.send_goal_async(goal)
                self.wait(sent.done, 15, '避车接收')
                handle = sent.result()
                assert handle.accepted
                self.handles.append(handle)
                result = handle.get_result_async()
                self.wait(lambda: self.right_edge or result.done(), 50, '靠右路径生成')
                assert self.right_edge and not result.done(), '未进入靠右分支'
                edge_start = self.robot()
                self.wait(lambda: result.done() or math.dist(edge_start[:2],self.robot()[:2]) > 0.25, 60, '实际靠右行驶')
                assert not result.done(), '靠右导航提前结束'
                pending = control.call_async(SetBool.Request(data=True))
                self.wait(pending.done, 10, '挂起后续找点')
                self.wait(lambda: any(e['event']=='held' for e in self.find_events), 10, '单次找点保持 RUNNING')
                # 先短暂消失 1 秒，再恢复，证明不会提前退出且计时重置。
                self.car_saved = self.car
                self.car = None
                self.empty_messages = mode == 'empty_array'
                until = self.now()+1.0
                self.wait(lambda: self.now() >= until or result.done(), 10, '短暂丢失')
                assert not result.done(), '不到 3 秒便提前退出'
                self.car = self.car_saved
                until = self.now()+1.0
                self.wait(lambda: self.now() >= until or result.done(), 10, '动车恢复')
                assert not result.done(), '恢复动车后仍错误退出'
                active_follow = {key for key, state in self.follow_states.items()
                                 if state == GoalStatus.STATUS_EXECUTING}
                assert active_follow, '消失前应有正在执行的 FollowPath'
                self.car = None
                gone = self.last_car_stamp
                self.log('动车消失', last_car_time=gone, held_find=True)
                self.wait(result.done, 15, '连续 3 秒无车退出')
                elapsed = self.now()-gone
                wrapped = result.result()
                assert wrapped.status == GoalStatus.STATUS_SUCCEEDED and wrapped.result.success, str(wrapped)
                assert 2.9-1e-6 <= elapsed <= 4.5, f'退出时间不符合 3 秒要求：{elapsed}'
                self.wait(lambda: any(e['event']=='canceled' for e in self.find_events), 5, '找点 Action 已取消')
                assert not any(e['event']=='hold_timeout' for e in self.find_events)
                self.wait(lambda: all(self.follow_states.get(key) == GoalStatus.STATUS_CANCELED
                                      for key in active_follow), 5, '靠右 FollowPath 已取消')
                self.log('避车退出', elapsed_seconds=elapsed, find_events=list(self.find_events),
                         follow_path_canceled=True)
                self.confirm_stop_command()
                # 上层重新下发同一个原目标，不能只凭避车 SUCCESS 判定恢复成功。
                _, resumed = self.send_nav(target)
                self.wait(resumed.done, 180, '继续完成原任务')
                assert resumed.result().status == GoalStatus.STATUS_SUCCEEDED
                pose = self.confirm_stop_command()
                assert math.dist(pose[:2], target[:2]) < 0.7
                self.log('原任务完成', pose=pose, target=target)
            self.report['result'] = 'PASS'
            self.save()

        def artifacts(self):
            (self.report_path.parent / 'commands.json').write_text(json.dumps(self.commands))
            (self.report_path.parent / "trajectory.json").write_text(json.dumps(self.trace, ensure_ascii=False))
    rclpy.init()
    node = Tester()
    code = 0
    try:
        node.execute()
        print("PASS：两棵原行为树，空数组/停发消息，3 秒退出、取消未完成找点、恢复原任务全部通过", flush=True)
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
