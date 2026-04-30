import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
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

    default_map = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps', 'cf_room2_map.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='room2_world.sdf',
        description='Gazebo world file to load',
    )

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Absolute path to saved raster map yaml file',
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

    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x', default_value='0.0',
        description='Initial X position on map (meters)',
    )
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='0.0',
        description='Initial Y position on map (meters)',
    )
    spawn_yaw_arg = DeclareLaunchArgument(
        'spawn_yaw', default_value='0.0',
        description='Initial heading (radians)',
    )

    crazyflie_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_bringup, 'launch', 'crazyflie_simulation.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_multiranger_bringup, 'config', 'amcl_localization_sim.yaml'),
            'use_composition': 'False',
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': os.path.join(pkg_multiranger_bringup, 'config', 'navigation_sim.yaml'),
            'use_composition': 'False',
        }.items(),
    )

    navigation_delayed = TimerAction(
        period=LaunchConfiguration('nav_delay'),
        actions=[navigation_launch],
    )

    localization_delayed = TimerAction(
        period=3.0,
        actions=[localization_launch],
    )

    # Bridge scan frame used by Gazebo sensors into the TF tree expected by AMCL/Nav2.
    scan_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='scan_static_tf_navigation',
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

    # Teleport crazyflie model in Gazebo to the requested spawn position.
    # Runs after takeoff (hover_height=0.2) so the drone is already airborne.
    set_gz_pose = ExecuteProcess(
        cmd=[
            'gz', 'service',
            '-s', '/world/default/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req',
            ['name: "crazyflie", position: {x: ',
             LaunchConfiguration('spawn_x'),
             ', y: ',
             LaunchConfiguration('spawn_y'),
             ', z: 0.2}'],
        ],
        output='log',
    )
    set_gz_pose_delayed = TimerAction(period=5.0, actions=[set_gz_pose])

    # Auto-publish initial pose matching spawn coordinates after AMCL is up
    initialpose_node = Node(
        package='crazyflie_ros2_multiranger_bringup',
        executable='send_initialpose.py',
        name='auto_initialpose',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': LaunchConfiguration('spawn_x'),
            'y': LaunchConfiguration('spawn_y'),
            'yaw': LaunchConfiguration('spawn_yaw'),
        }],
        output='log',
    )
    initialpose_delayed = TimerAction(period=12.0, actions=[initialpose_node])

    return LaunchDescription([
        world_arg,
        map_arg,
        rviz_arg,
        use_sim_time_arg,
        nav_delay_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        crazyflie_simulation,
        scan_static_tf,
        set_gz_pose_delayed,
        localization_delayed,
        navigation_delayed,
        rviz_delayed,
        initialpose_delayed,
    ])
