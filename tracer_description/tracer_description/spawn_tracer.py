#!/usr/bin/env python3
import sys
import rclpy

from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from functools import partial

class SpawnDeliverybotNode(Node):
    def __init__(self):
        super().__init__("minimal_client")
        client = self.create_client(SpawnEntity, '/spawn_entity')
    
        content = sys.argv[1]

        req = SpawnEntity.Request()
        req.name = "tracer"
        req.xml = content
        req.robot_namespace = "tracer"
        req.reference_frame = "world"
        req.initial_pose.position.x = 0.0
        req.initial_pose.position.y = 0.0
        req.initial_pose.position.z = 0.2

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Service not available, waiting again...')

        future = client.call_async(req)
        future.add_done_callback(partial(self.callback_spawn_entity))

    def callback_spawn_entity(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = SpawnDeliverybotNode()
    rclpy.spin(node)
    rclpy.shutdown()
 