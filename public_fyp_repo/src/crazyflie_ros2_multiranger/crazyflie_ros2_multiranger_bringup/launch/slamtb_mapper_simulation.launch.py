import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='crazyflie_world.sdf',
        description='Gazebo world file to load',
    )

    pkg_project_crazyflie_gazebo = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_multiranger_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')

    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_crazyflie_gazebo, 'launch', 'crazyflie_simulation.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_multiranger_bringup, 'config', 'slam_params_sim.yaml'),
            'use_sim_time': 'True',
        }.items(),
    )

    rviz_config_path = os.path.join(pkg_multiranger_bringup, 'config', 'slam_mapping_sim.rviz')

    # Bridge the lidar frame to the robot base frame for slam_toolbox TF lookups.
    # The scan frame comes from Gazebo as crazyflie/crazyflie/body/multiranger.
    scan_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_static_tf',
        arguments=['0', '0', '0.1', '0', '0', '0',
                   'crazyflie/base_footprint', 'crazyflie/crazyflie/body/multiranger'],
    )

    rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        crazyflie_simulation,
        scan_static_tf,
        slam_toolbox,
        rviz,
    ])
