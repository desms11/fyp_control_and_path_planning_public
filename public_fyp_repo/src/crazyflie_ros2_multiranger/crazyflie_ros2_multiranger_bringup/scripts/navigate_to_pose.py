#!/usr/bin/env python3
"""
Navigate drone from current position to a goal using one of four algorithms.

Supported planners:
  Dijkstra  Nav2 NavfnPlanner (optimal, default)
  AStar     Nav2 NavfnPlanner with A* heuristic
  RRT       Rapidly-exploring Random Tree (custom)
  GA        Genetic Algorithm (custom)

Each run produces a JSON report in ~/ros2_ws/reports/.

Usage:
  ros2 run crazyflie_ros2_multiranger_bringup navigate_to_pose.py \
      --ros-args -p use_sim_time:=true -p goal_x:=2.0 -p goal_y:=1.0 \
      -p planner:=Dijkstra
"""

import json
import math
import os
import sys
import time
import uuid
from datetime import datetime

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav2_msgs.action import NavigateToPose, FollowPath
from lifecycle_msgs.srv import GetState

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from planners import RRTPlanner, GAPlanner  # noqa: E402

VALID_PLANNERS = ['Dijkstra', 'AStar', 'RRT', 'GA']
REPORTS_DIR = os.path.expanduser('~/ros2_ws/reports')


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


def path_length_meters(poses):
    """Sum of Euclidean distances between consecutive PoseStamped."""
    length = 0.0
    for i in range(1, len(poses)):
        a = poses[i - 1].pose.position
        b = poses[i].pose.position
        length += math.hypot(b.x - a.x, b.y - a.y)
    return length


def path_smoothness(poses):
    """Average absolute turning angle in radians (lower = smoother)."""
    if len(poses) < 3:
        return 0.0
    angles = []
    for i in range(1, len(poses) - 1):
        ax = poses[i].pose.position.x - poses[i - 1].pose.position.x
        ay = poses[i].pose.position.y - poses[i - 1].pose.position.y
        bx = poses[i + 1].pose.position.x - poses[i].pose.position.x
        by = poses[i + 1].pose.position.y - poses[i].pose.position.y
        a1 = math.atan2(ay, ax)
        a2 = math.atan2(by, bx)
        diff = abs(a2 - a1)
        if diff > math.pi:
            diff = 2 * math.pi - diff
        angles.append(diff)
    return float(np.mean(angles)) if angles else 0.0


class Nav2GoalSender(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_client')

        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('planner', 'Dijkstra')
        self.declare_parameter('wait_for_nav2', True)

        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.goal_yaw = self.get_parameter('goal_yaw').value
        self.planner_name = self.get_parameter('planner').value
        self.wait_for_nav2 = self.get_parameter('wait_for_nav2').value

        if self.planner_name not in VALID_PLANNERS:
            self.get_logger().error(
                f'Unknown planner "{self.planner_name}". '
                f'Valid: {VALID_PLANNERS}')
            raise SystemExit(1)

        # Action clients
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._follow_client = ActionClient(self, FollowPath, 'follow_path')

        # Path publisher for RViz visualization of custom planner paths
        self._path_pub = self.create_publisher(Path, '/plan', 10)

        # Odom tracking for actual trajectory
        self._odom_positions = []
        self._odom_sub = self.create_subscription(
            Odometry, '/crazyflie/odom', self._odom_cb, 10)

        # Map data for custom planners
        self._map_data = None
        self._map_info = None
        if self.planner_name in ('RRT', 'GA'):
            map_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=1)
            self._map_sub = self.create_subscription(
                OccupancyGrid, '/map', self._map_cb, map_qos)

        # Planned path for Nav2 planners (captured from /plan topic for Dijkstra/A*)
        self._planned_path_poses = []
        if self.planner_name in ('Dijkstra', 'AStar'):
            self._plan_sub = self.create_subscription(
                Path, '/plan', self._plan_cb, 10)

        # BT XML for A*
        if self.planner_name == 'AStar':
            pkg = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
            self.bt_xml = os.path.join(pkg, 'config', 'nav_to_pose_astar.xml')
        else:
            self.bt_xml = ''

        # Timing / state
        self.result_received = False
        self.success = False
        self._start_pose = None
        self._planning_start = None
        self._planning_time = None
        self._exec_start = None
        self._total_start = None
        self._num_recoveries = 0

    # ── Callbacks ──

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self._odom_positions.append((p.x, p.y))

    def _map_cb(self, msg):
        w = msg.info.width
        h = msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        self._map_data = data
        self._map_info = msg.info
        self.get_logger().info(f'Map received: {w}x{h}, res={msg.info.resolution}')

    def _plan_cb(self, msg):
        if not self._planned_path_poses:
            self._planned_path_poses = list(msg.poses)
        if self._planning_time is None and self._planning_start is not None:
            self._planning_time = time.monotonic() - self._planning_start

    # ── Nav2 readiness ──

    def wait_for_nav2_active(self):
        srv_name = '/bt_navigator/get_state'
        client = self.create_client(GetState, srv_name)
        self.get_logger().info('Waiting for bt_navigator to become active...')
        while not client.wait_for_service(timeout_sec=2.0):
            if not rclpy.ok():
                return False
            self.get_logger().info(f'  ...{srv_name} not yet available')
        while rclpy.ok():
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None:
                label = future.result().current_state.label
                if label == 'active':
                    self.get_logger().info('bt_navigator is active.')
                    return True
                self.get_logger().info(f'  bt_navigator state: {label}')
            time.sleep(1.0)
        return False

    # ── World ↔ Grid conversion ──

    def _world_to_grid(self, wx, wy):
        info = self._map_info
        col = int((wx - info.origin.position.x) / info.resolution)
        row = int((wy - info.origin.position.y) / info.resolution)
        return (row, col)

    def _grid_to_world(self, row, col):
        info = self._map_info
        wx = info.origin.position.x + (col + 0.5) * info.resolution
        wy = info.origin.position.y + (row + 0.5) * info.resolution
        return (wx, wy)

    # ── Get current robot pose from odom ──

    def _get_current_pose(self):
        if self._odom_positions:
            return self._odom_positions[-1]
        return (0.0, 0.0)

    # ── Send goal: Dijkstra / A* via NavigateToPose ──

    def send_goal_nav2(self):
        self.get_logger().info('Waiting for navigate_to_pose server...')
        if not self._nav_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('NavigateToPose server unavailable!')
            self.result_received = True
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self.goal_x)
        goal.pose.pose.position.y = float(self.goal_y)
        qx, qy, qz, qw = yaw_to_quaternion(float(self.goal_yaw))
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        goal.behavior_tree = self.bt_xml

        self._start_pose = self._get_current_pose()
        self._total_start = time.monotonic()
        self._planning_start = time.monotonic()
        self._odom_positions.clear()

        self.get_logger().info(
            f'Goal: ({self.goal_x}, {self.goal_y}), planner={self.planner_name}')

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._nav2_feedback)
        future.add_done_callback(self._nav2_goal_response)

    def _nav2_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal REJECTED')
            self.result_received = True
            return
        self.get_logger().info('Goal accepted — navigating...')
        self._exec_start = time.monotonic()
        gh.get_result_async().add_done_callback(self._nav2_result)

    def _nav2_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self._num_recoveries = fb.number_of_recoveries
        cur = fb.current_pose.pose.position
        self.get_logger().info(
            f'  pos=({cur.x:.2f},{cur.y:.2f}) '
            f'remaining={fb.distance_remaining:.2f}m')

    def _nav2_result(self, future):
        status = future.result().status
        self.success = (status == GoalStatus.STATUS_SUCCEEDED)
        label = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
        }.get(status, f'UNKNOWN({status})')
        self.get_logger().info(f'Navigation finished: {label}')
        self._generate_report(label)
        self.result_received = True

    # ── Send goal: RRT / GA via custom planner + FollowPath ──

    def send_goal_custom(self):
        self.get_logger().info(f'Running {self.planner_name} planner...')

        # Wait for map
        deadline = time.monotonic() + 15.0
        while self._map_data is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        if self._map_data is None:
            self.get_logger().error('No map received!')
            self.result_received = True
            return

        # Wait for at least one odom message
        deadline = time.monotonic() + 5.0
        while not self._odom_positions and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)

        self._start_pose = self._get_current_pose()
        start_cell = self._world_to_grid(self._start_pose[0], self._start_pose[1])
        goal_cell = self._world_to_grid(self.goal_x, self.goal_y)

        h, w = self._map_data.shape
        start_cell = (np.clip(start_cell[0], 0, h - 1), np.clip(start_cell[1], 0, w - 1))
        goal_cell = (np.clip(goal_cell[0], 0, h - 1), np.clip(goal_cell[1], 0, w - 1))

        self.get_logger().info(
            f'  start_cell={start_cell}, goal_cell={goal_cell}, '
            f'grid={h}x{w}')

        # Run planner
        self._total_start = time.monotonic()
        self._planning_start = time.monotonic()

        if self.planner_name == 'RRT':
            planner = RRTPlanner()
            cell_path, info = planner.plan(self._map_data, start_cell, goal_cell)
        else:
            planner = GAPlanner()
            cell_path, info = planner.plan(self._map_data, start_cell, goal_cell)

        self._planning_time = time.monotonic() - self._planning_start
        self.get_logger().info(
            f'  {self.planner_name} planning took {self._planning_time:.3f}s '
            f'info={info}')

        if cell_path is None:
            self.get_logger().error(f'{self.planner_name} failed to find a path!')
            self._generate_report('PLANNING_FAILED')
            self.result_received = True
            return

        # Convert to ROS Path
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for row, col in cell_path:
            wx, wy = self._grid_to_world(row, col)
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self._planned_path_poses = list(path_msg.poses)
        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f'  Published path with {len(path_msg.poses)} poses')

        # Send to FollowPath controller
        self.get_logger().info('Waiting for follow_path server...')
        if not self._follow_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error('FollowPath server unavailable!')
            self.result_received = True
            return

        fp_goal = FollowPath.Goal()
        fp_goal.path = path_msg
        fp_goal.controller_id = 'FollowPath'
        fp_goal.goal_checker_id = 'general_goal_checker'

        self._odom_positions.clear()
        self._exec_start = time.monotonic()

        future = self._follow_client.send_goal_async(
            fp_goal, feedback_callback=self._follow_feedback)
        future.add_done_callback(self._follow_goal_response)

    def _follow_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('FollowPath goal REJECTED')
            self._generate_report('REJECTED')
            self.result_received = True
            return
        self.get_logger().info('FollowPath goal accepted — executing...')
        gh.get_result_async().add_done_callback(self._follow_result)

    def _follow_feedback(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(
            f'  dist_to_goal={fb.distance_to_goal:.2f}m '
            f'speed={fb.speed:.2f}m/s')

    def _follow_result(self, future):
        result = future.result().result
        error_code = result.error_code if hasattr(result, 'error_code') else 0
        self.success = (error_code == 0)
        label = 'SUCCEEDED' if self.success else f'FAILED(err={error_code})'
        self.get_logger().info(f'FollowPath finished: {label}')
        self._generate_report(label)
        self.result_received = True

    # ── Report generation ──

    def _generate_report(self, status_label):
        total_time = time.monotonic() - self._total_start if self._total_start else 0
        exec_time = time.monotonic() - self._exec_start if self._exec_start else 0
        planning_time = self._planning_time or 0.0

        planned_length = path_length_meters(self._planned_path_poses)
        smoothness = path_smoothness(self._planned_path_poses)

        actual_dist = 0.0
        positions = self._odom_positions
        for i in range(1, len(positions)):
            actual_dist += math.hypot(
                positions[i][0] - positions[i - 1][0],
                positions[i][1] - positions[i - 1][1])

        efficiency = (planned_length / actual_dist) if actual_dist > 1e-3 else 0.0
        avg_speed = actual_dist / exec_time if exec_time > 0.1 else 0.0

        start = self._start_pose or (0.0, 0.0)

        report = {
            'run_id': str(uuid.uuid4())[:8],
            'timestamp': datetime.now().isoformat(),
            'algorithm': self.planner_name,
            'map_file': 'cf_room2_map.yaml',
            'start_pose': {'x': round(start[0], 3), 'y': round(start[1], 3)},
            'goal_pose': {'x': self.goal_x, 'y': self.goal_y,
                          'yaw': self.goal_yaw},
            'status': status_label,
            'metrics': {
                'path_length_planned_m': round(planned_length, 4),
                'distance_traveled_m': round(actual_dist, 4),
                'planning_time_s': round(planning_time, 4),
                'execution_time_s': round(exec_time, 4),
                'total_time_s': round(total_time, 4),
                'completeness': 1 if self.success else 0,
                'energy_efficiency': round(efficiency, 4),
                'path_smoothness_rad': round(smoothness, 4),
                'num_recoveries': self._num_recoveries,
                'avg_velocity_ms': round(avg_speed, 4),
            },
        }

        os.makedirs(REPORTS_DIR, exist_ok=True)
        filename = f'{self.planner_name}_{report["run_id"]}.json'
        filepath = os.path.join(REPORTS_DIR, filename)
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2)

        self.get_logger().info(f'Report saved: {filepath}')
        self.get_logger().info(
            f'  planned_path={planned_length:.2f}m  '
            f'traveled={actual_dist:.2f}m  '
            f'planning={planning_time:.3f}s  '
            f'total={total_time:.1f}s  '
            f'status={status_label}')


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalSender()

    if node.wait_for_nav2:
        if not node.wait_for_nav2_active():
            node.get_logger().error('Nav2 not ready.')
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    if node.planner_name in ('Dijkstra', 'AStar'):
        node.send_goal_nav2()
    else:
        node.send_goal_custom()

    while rclpy.ok() and not node.result_received:
        rclpy.spin_once(node, timeout_sec=0.5)

    rc = 0 if node.success else 1
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
