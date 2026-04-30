#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
import math

def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    q = [0]*4
    q[0] = sr * cp * cy - cr * sp * sy #x
    q[1] = cr * sp * cy + sr * cp * sy #y
    q[2] = cr * cp * sy - sr * sp * cy #z
    q[3] = cr * cp * cy + sr * sp * sy #w
    return q

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower_script')
        self._action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
    def define_waypoints(self):
        waypoints = []

        # Waypoint 1: Go forward in the room
        wp1 = PoseStamped()
        wp1.header.frame_id = 'map'
        wp1.pose.position.x = 2.0
        wp1.pose.position.y = 0.0
        q = euler_to_quaternion(0, 0, 0)
        wp1.pose.orientation.x = q[0]
        wp1.pose.orientation.y = q[1]
        wp1.pose.orientation.z = q[2]
        wp1.pose.orientation.w = q[3]
        waypoints.append(wp1)

        # Waypoint 2: Turn and go to the side
        wp2 = PoseStamped()
        wp2.header.frame_id = 'map'
        wp2.pose.position.x = 2.0
        wp2.pose.position.y = 2.0
        q = euler_to_quaternion(0, 0, 1.57) # 90 degrees
        wp2.pose.orientation.x = q[0]
        wp2.pose.orientation.y = q[1]
        wp2.pose.orientation.z = q[2]
        wp2.pose.orientation.w = q[3]
        waypoints.append(wp2)

        # Waypoint 3: Return near start
        wp3 = PoseStamped()
        wp3.header.frame_id = 'map'
        wp3.pose.position.x = 0.0
        wp3.pose.position.y = 0.0
        q = euler_to_quaternion(0, 0, 3.14) # 180 degrees
        wp3.pose.orientation.x = q[0]
        wp3.pose.orientation.y = q[1]
        wp3.pose.orientation.z = q[2]
        wp3.pose.orientation.w = q[3]
        waypoints.append(wp3)

        return waypoints

    def send_goal(self):
        waypoints = self.define_waypoints()
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = waypoints

        self.get_logger().info('Waiting for follow_waypoints action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Sending waypoints...')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected.')
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        self.get_logger().info(f'Navigating to waypoint {current_waypoint}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Waypoint following completed.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    waypoint_follower = WaypointFollower()
    waypoint_follower.send_goal()
    rclpy.spin(waypoint_follower)

if __name__ == '__main__':
    main()
