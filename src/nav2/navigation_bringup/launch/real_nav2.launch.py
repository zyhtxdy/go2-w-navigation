import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_share = get_package_share_directory('navigation_bringup')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    navigate_to_pose_bt = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_to_pose_w_replanning_no_spin.xml',
    )
    navigate_through_poses_bt = os.path.join(
        package_share,
        'behavior_trees',
        'navigate_through_poses_w_replanning_no_spin.xml',
    )

    # Keep the YAML portable between the laptop and NX: rewrite the installed
    # behavior-tree path at launch time instead of hard-coding either home.
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': navigate_to_pose_bt,
            'default_nav_through_poses_bt_xml': navigate_through_poses_bt,
            'lio_height_offset': LaunchConfiguration('lio_height_offset'),
        },
        convert_types=True,
    )

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            )
        ),
        launch_arguments={
            'params_file': configured_params,
            'use_sim_time': use_sim_time,
            # Humble evaluates PythonExpression(['not ', use_composition]);
            # this value must be the Python literal False, not lowercase false.
            'use_composition': 'False',
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )
    # Humble controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel.
    # Remap only the smoother's OUTPUT and the behavior server's OUTPUT:
    # a global cmd_vel remap would also redirect the smoother's input and could
    # bypass the smoothing stage. The private topic avoids others' /cmd_vel.
    nav2_with_velocity_output = GroupAction(actions=[
        SetRemap(src='velocity_smoother:cmd_vel_smoothed', dst=cmd_vel_topic),
        SetRemap(src='behavior_server:cmd_vel', dst=cmd_vel_topic),
        nav2_navigation,
    ])
    velocity_bridge = Node(
        package='go2_cmd_vel_bridge',
        executable='cmd_vel_bridge',
        name='go2_cmd_vel_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_cmd_vel_bridge')),
        parameters=[
            os.path.join(get_package_share_directory('go2_cmd_vel_bridge'),
                         'config', 'go2_cmd_vel.yaml'),
            {
                'input_topic': cmd_vel_topic,
                'request_topic': LaunchConfiguration('sport_request_topic'),
                'enabled': ParameterValue(LaunchConfiguration('bridge_enabled'), value_type=bool),
                'forward_compensation_enabled': ParameterValue(
                    LaunchConfiguration('forward_compensation'), value_type=bool),
                'angular_compensation_enabled': ParameterValue(
                    LaunchConfiguration('angular_compensation'), value_type=bool),
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_share, 'config', 'nav2_mppi_params.yaml'),
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('start_cmd_vel_bridge', default_value='true'),
        DeclareLaunchArgument('bridge_enabled', default_value='true'),
        DeclareLaunchArgument('forward_compensation', default_value='true',
                             description='Enable empirical positive-forward speed feedforward.'),
        DeclareLaunchArgument('angular_compensation', default_value='true',
                             description='Symmetric yaw feedforward based on the measured positive response.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/go2/nav/cmd_vel'),
        DeclareLaunchArgument('sport_request_topic', default_value='/api/sport/request'),
        DeclareLaunchArgument(
            'lio_height_offset', default_value='-0.50',
            description='Nominal IMU-to-ground Z offset in metres; measure for the operating posture.'),
        nav2_with_velocity_output,
        velocity_bridge,
    ])
