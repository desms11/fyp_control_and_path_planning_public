import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_multiranger_bringup = get_package_share_directory('crazyflie_ros2_multiranger_bringup')
    pkg_ros_gz_bringup = get_package_share_directory('ros_gz_crazyflie_bringup')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='crazyflie_world.sdf',
        description='Gazebo world file to load',
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz for navigation',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock',
    )

    nav_delay_arg = DeclareLaunchArgument(
        'nav_delay',
        default_value='8.0',
        description='Delay (seconds) before starting Nav2 navigation stack',
    )

    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_bringup, 'launch', 'crazyflie_simulation.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(pkg_multiranger_bringup, 'config', 'slam_params_sim.yaml'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_multiranger_bringup, 'config', 'navigation_sim.yaml'),
            'use_composition': 'True',
            'container_name': 'nav2_container',
        }.items(),
    )

    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}, {'autostart': True}],
        arguments=['--ros-args', '--log-level', 'info'],
    )

    navigation_delayed = TimerAction(
        period=LaunchConfiguration('nav_delay'),
        actions=[navigation_launch],
    )

    # Bridge scan frame used by Gazebo sensors into the TF tree expected by slam_toolbox/Nav2.
    scan_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_static_tf_navigation_slam',
        arguments=['0', '0', '0.1', '0', '0', '0',
                   'crazyflie/base_footprint', 'crazyflie/crazyflie/body/multiranger'],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'rviz_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('rviz')),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'rviz_config': os.path.join(pkg_nav2_bringup, 'rviz', 'nav2_default_view.rviz'),
        }.items(),
    )

    rviz_delayed = TimerAction(period=8.0, actions=[rviz])

    return LaunchDescription([
        world_arg,
        rviz_arg,
        use_sim_time_arg,
        nav_delay_arg,
        crazyflie_simulation,
        scan_static_tf,
        nav2_container,
        slam_toolbox,
        navigation_delayed,
        rviz_delayed,
    ])
