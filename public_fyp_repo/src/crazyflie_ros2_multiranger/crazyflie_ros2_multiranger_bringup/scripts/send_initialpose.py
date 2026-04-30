#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import PoseWithCovarianceStamped


def yaw_to_quaternion(yaw):
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        # dynamic_typing allows integer overrides for float parameters
        # (e.g. spawn_yaw:=0 instead of spawn_yaw:=0.0)
        flex = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('x', 0.0, flex)
        self.declare_parameter('y', 0.0, flex)
        self.declare_parameter('yaw', 0.0, flex)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_duration', 5.0, flex)
        self.declare_parameter('publish_rate', 5.0, flex)

        self.x = float(self.get_parameter('x').value)
        self.y = float(self.get_parameter('y').value)
        self.yaw = float(self.get_parameter('yaw').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_duration = float(self.get_parameter('publish_duration').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', qos)

        self.publish_count = 0
        self.max_publishes = int(self.publish_duration * self.publish_rate)
        self.done = False
        self.subscriber_found = False

        self.get_logger().info(
            f'Waiting for subscribers on /initialpose before publishing '
            f'({self.x}, {self.y}, yaw={self.yaw})...')

        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._tick)

    def _build_msg(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(float(self.yaw))
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.07,
        ]
        return msg

    def _tick(self):
        sub_count = self.publisher.get_subscription_count()

        if not self.subscriber_found:
            if sub_count == 0:
                return
            self.subscriber_found = True
            self.get_logger().info(
                f'Found {sub_count} subscriber(s), publishing initial pose...')

        msg = self._build_msg()
        self.publisher.publish(msg)
        self.publish_count += 1

        if self.publish_count >= self.max_publishes:
            self.get_logger().info(
                f'Published initial pose {self.publish_count} times. Done.')
            self.done = True
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
