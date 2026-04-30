#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory("crazyflie_ros2_multiranger_bringup")

    # Inherit the "Navigation with SLAM" standalone launch
    navigation_with_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_with_slam_standalone.launch.py")
        )
    )

    # Explore Lite node
    explore_lite_node = Node(
        package='explore_lite',
        executable='explore',
        name='explore_node',
        output='screen',
        parameters=[{
            'robot_base_frame': 'crazyflie/base_footprint',
            'costmap_topic': '/global_costmap/costmap',
            'costmap_updates_topic': '/global_costmap/costmap_updates',
            'visualize': True,
            'planner_frequency': 1.0,
            'progress_timeout': 30.0,
            'potential_scale': 3.0,
            'orientation_scale': 0.0,
            'gain_scale': 1.0,
            'transform_tolerance': 0.3,
            'min_frontier_size': 0.5,
            'return_to_init': False,
        }]
    )

    # Delay exploration start to let Nav2 and SLAM fully initialize
    explore_delayed = TimerAction(
        period=20.0,
        actions=[explore_lite_node]
    )

    return LaunchDescription([
        navigation_with_slam_launch,
        explore_delayed,
    ])
