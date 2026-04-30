#!/usr/bin/env python3
"""
Navigation simulation launch using STANDALONE (non-composable) nodes.
Integrated with MOGI-ROS Jazzy parameters.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch CrazyFlie simulation with standalone Nav2 stack."""

    # Package directories
    sim_bringup_dir = get_package_share_directory("ros_gz_crazyflie_bringup")
    bringup_dir = get_package_share_directory("crazyflie_ros2_multiranger_bringup")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # Launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_arg = LaunchConfiguration("world", default="room2_world.sdf")
    
    # Standard map from workspace - fall back to the safe, downloaded one if default is broken
    map_arg = LaunchConfiguration(
        "map",
        default="/home/ubunuser/gazebo_models/experiment_rooms_subset/worlds/experiment_rooms/worlds/room2/map/room2_ros_aligned.yaml",
    )
    
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(bringup_dir, "config", "navigation_sim.yaml"),
    )

    # 1. Simulation + control bringup
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_bringup_dir, "launch", "crazyflie_simulation.launch.py")
        ),
        launch_arguments={"world": world_arg}.items(),
    )

    # 2. Localization (AMCL) with MapServer
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "localization_launch.py")
        ),
        launch_arguments={
            "map": map_arg,
            "use_sim_time": use_sim_time,
            "autostart": "True",
            "params_file": os.path.join(bringup_dir, "config", "amcl_localization_sim.yaml"),
        }.items(),
    )

    # 3. Navigation (standalone nodes, not composable)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "True",
            "use_composition": "False",
        }.items(),
    )

    # Bridge scan frame
    scan_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_static_tf_navigation',
        arguments=['0', '0', '0.1', '0', '0', '0',
                   'crazyflie/base_footprint', 'crazyflie/crazyflie/body/multiranger'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(
        [
            simulation_launch,
            scan_static_tf,
            TimerAction(period=3.0, actions=[localization_launch]),
            TimerAction(period=12.0, actions=[navigation_launch]),
        ]
    )
