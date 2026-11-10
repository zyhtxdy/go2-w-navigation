"""Replay recorded inputs through mapping, fusion, terrain and the global planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction, UnsetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory('fast_lio_nav2')
    parameters = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={
            'use_sim_time': 'true',
            'lio_height_offset': LaunchConfiguration('lio_height_offset'),
        },
        convert_types=True,
    )
    player = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', LaunchConfiguration('bag'),
            '--clock', '100', '--rate', LaunchConfiguration('rate'), '--start-paused',
            '--topics', '/hesai/points', '/go2/imu',
        ], output='screen',
    )

    def finish_timeline(event, context):
        del context
        if event.returncode != 0:
            return []
        return [TimerAction(period=.5, actions=[ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/replay_starter/finish',
                 'std_srvs/srv/Trigger', '{}'], output='screen',
        )])]

    return LaunchDescription([
        DeclareLaunchArgument('bag', description='Complete recorded bag directory.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'nav2_mppi_params.yaml'),
            description='The same global-planner parameters used on the robot.',
        ),
        DeclareLaunchArgument('rate', default_value='1.0'),
        DeclareLaunchArgument('ros_domain_id', default_value='77'),
        DeclareLaunchArgument('lio_height_offset', default_value='-0.50'),
        # Keep replay publishers and services separate from the live robot.
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        UnsetEnvironmentVariable('CYCLONEDDS_URI'),
        UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER'),
        UnsetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'odom_fusion.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'lidar_driver': 'none',
                'start_go2_imu_bridge': 'false',
                'start_go2_odom_bridge': 'false',
            }.items(),
        ),
        # Read raw state CDR locally and rebuild body velocity. The recorded
        # /go2/odom_raw contains the former, incorrect extra frame rotation.
        # Avoid publishing the native recorded firmware layout through DDS.
        Node(
            package='go2_odom_bridge', executable='sportmodestate_odom_bridge',
            name='go2_odom_bag_bridge', output='screen',
            parameters=[{
                'bag_path': LaunchConfiguration('bag'),
                'input_topic': '/lf/sportmodestate',
                'output_topic': '/go2/odom_raw',
                'velocity_frame': 'body',
                'timestamp_source': 'receive',
                'use_sim_time': True,
            }],
        ),
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[parameters],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='offline_planner_lifecycle_manager', output='screen',
            parameters=[{
                'use_sim_time': True, 'autostart': True,
                'node_names': ['planner_server'],
            }],
        ),
        Node(
            package='fast_lio_nav2', executable='rviz_goal_planner.py',
            name='rviz_goal_planner', output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='fast_lio_nav2', executable='replay_starter.py',
            name='replay_starter', output='screen',
            parameters=[{'bag_path': LaunchConfiguration('bag'), 'use_sim_time': True}],
        ),
        # Delay process startup, not rosbag playback. rosbag --delay publishes
        # an advancing preview clock, then rewinds when sensor playback starts.
        # The raw-state bridge must never consume records against that clock.
        RegisterEventHandler(OnProcessExit(target_action=player, on_exit=finish_timeline)),
        TimerAction(period=3.0, actions=[player]),
    ])
