import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    pkg_multiranger_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    pkg_ros_gz_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='room2_world.sdf',
        description='Gazebo world file to load',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz for SLAM localization',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='localization_sim.rviz',
        description='RViz config file in crazyflie_ros2_multiranger_bringup/config',
    )

    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_bringup, 'launch', 'crazyflie_simulation.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    slam_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_multiranger_bringup, 'config', 'slam_params_localization_sim.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Bridge scan frame used by Gazebo sensors into the TF tree expected by SLAM Toolbox.
    scan_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_static_tf_slam_localization',
        arguments=['0', '0', '0.1', '0', '0', '0',
                   'crazyflie/base_footprint', 'crazyflie/crazyflie/body/multiranger'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam_localization',
        arguments=['-d', PathJoinSubstitution([pkg_multiranger_bringup, 'config', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz_delayed = TimerAction(period=8.0, actions=[rviz])

    return LaunchDescription([
        world_arg,
        rviz_arg,
        use_sim_time_arg,
        rviz_config_arg,
        crazyflie_simulation,
        scan_static_tf,
        slam_localization,
        rviz_delayed,
    ])
