#!/usr/bin/env python3
"""仅测试用：无车位时返回失败；收到控制信号后挂起，直到真实 BT 取消 Action。"""
import json
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from capella_ros_msg.action import FindCarAvoidancePoint
from std_msgs.msg import String
from std_srvs.srv import SetBool

class Server(Node):
    def __init__(self):
        super().__init__('test2_controlled_find')
        self.hold=threading.Event()
        self.group=ReentrantCallbackGroup()
        self.pub=self.create_publisher(String,'/test2/find_events',100)
        self.create_service(SetBool,'/test2/hold_find',self.control,callback_group=self.group)
        self.server=ActionServer(self,FindCarAvoidancePoint,'/find_car_avoidance_point_action',
            execute_callback=self.execute,cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=self.group)
    def event(self,name):
        self.pub.publish(String(data=json.dumps({'event':name,'wall_time':time.monotonic()})))
    def control(self,req,res):
        self.hold.set() if req.data else self.hold.clear()
        res.success=True
        return res
    def execute(self,goal):
        result=FindCarAvoidancePoint.Result()
        if not self.hold.is_set():
            goal.abort();self.event('no_point');return result
        self.event('held')
        deadline=time.monotonic()+90
        while rclpy.ok() and time.monotonic()<deadline:
            if goal.is_cancel_requested:
                goal.canceled();self.event('canceled');return result
            time.sleep(.01)
        goal.abort();self.event('hold_timeout');return result

rclpy.init();node=Server();executor=MultiThreadedExecutor(num_threads=3)
try:rclpy.spin(node,executor=executor)
except KeyboardInterrupt:pass
finally:
    executor.shutdown();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
