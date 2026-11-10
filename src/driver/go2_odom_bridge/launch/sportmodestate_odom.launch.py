from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'input_topic': '/lf/sportmodestate',
        'output_topic': '/go2/odom_raw',
        'frame_id': 'go2_state_odom',
        'child_frame_id': 'go2_body',
        'velocity_frame': 'body',
        'timestamp_source': 'receive',
        'linear_velocity_stddev': '0.2',
        'queue_depth': '20',
        'use_sim_time': 'false',
    }
    arguments = [DeclareLaunchArgument(name, default_value=value)
                 for name, value in defaults.items()]
    parameters = {name: LaunchConfiguration(name) for name in (
        'input_topic', 'output_topic', 'frame_id', 'child_frame_id',
        'velocity_frame', 'timestamp_source')}
    for name, value_type in (('linear_velocity_stddev', float),
                             ('queue_depth', int), ('use_sim_time', bool)):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=value_type)
    return LaunchDescription(arguments + [Node(
        package='go2_odom_bridge', executable='sportmodestate_odom_bridge',
        name='go2_odom_bridge', parameters=[parameters], output='screen',
    )])
