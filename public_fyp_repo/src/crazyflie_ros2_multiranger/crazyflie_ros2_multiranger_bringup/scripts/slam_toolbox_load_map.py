#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from slam_toolbox.srv import DeserializePoseGraph


class SlamMapLoader(Node):
    def __init__(self):
        super().__init__('slam_map_loader')

        default_map = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps', 'cf_room2_map_serialized')
        self.declare_parameter('map_file', default_map)
        self.declare_parameter('match_type', 1)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('theta', 0.0)

        map_file = self.get_parameter('map_file').value
        match_type = int(self.get_parameter('match_type').value)

        initial_pose = Pose2D()
        initial_pose.x = float(self.get_parameter('x').value)
        initial_pose.y = float(self.get_parameter('y').value)
        initial_pose.theta = float(self.get_parameter('theta').value)

        self.client = self.create_client(DeserializePoseGraph, 'slam_toolbox/deserialize_map')
        self.get_logger().info('Waiting for slam_toolbox/deserialize_map service...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available yet, retrying...')

        self.req = DeserializePoseGraph.Request()
        self.req.filename = str(map_file)
        self.req.match_type = match_type
        self.req.initial_pose = initial_pose

    def send_request(self):
        self.get_logger().info('Requesting map deserialize from: %s' % self.req.filename)
        return self.client.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)
    node = SlamMapLoader()
    future = node.send_request()
    rclpy.spin_until_future_complete(node, future)

    if future.result() is None:
        node.get_logger().error('Failed to deserialize map.')
    else:
        node.get_logger().info('SLAM map deserialize request completed.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
