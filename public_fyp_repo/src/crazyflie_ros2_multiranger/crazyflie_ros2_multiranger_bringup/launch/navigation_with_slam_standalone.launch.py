#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    sim_bringup_dir = get_package_share_directory("ros_gz_crazyflie_bringup")
    bringup_dir = get_package_share_directory("crazyflie_ros2_multiranger_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    slam_toolbox_dir = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_arg = LaunchConfiguration("world", default="room2_world.sdf")
    params_file = os.path.join(bringup_dir, "config", "navigation_sim.yaml")
    slam_params_file = os.path.join(bringup_dir, "config", "slam_params_sim.yaml")

    return LaunchDescription([
        # 1. Simulation Bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_bringup_dir, "launch", "crazyflie_simulation.launch.py")),
            launch_arguments={"world": world_arg}.items(),
        ),
        
        # 2. TF for Scan
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='scan_static_tf_navigation',
            arguments=['0', '0', '0.1', '0', '0', '0', 'crazyflie/base_footprint', 'crazyflie/crazyflie/body/multiranger'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # 3. SLAM Toolbox
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')),
                launch_arguments={
                    'slam_params_file': slam_params_file,
                    'use_sim_time': use_sim_time,
                }.items(),
            )
        ]),
        
        # 4. Navigation (Standalone Nodes)
        TimerAction(period=12.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": "True",
                    "use_composition": "False",
                }.items(),
            )
        ]),
        
        # 5. RViz (using the default nav2 config)
        TimerAction(period=15.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'rviz_config': os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz'),
                }.items(),
            )
        ]),
    ])
