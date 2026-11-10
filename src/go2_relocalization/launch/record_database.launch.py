from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output', description='New directory to export on save service.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='go2_relocalization', executable='database_recorder',
             name='relocalization_recorder', output='screen', parameters=[{
                 'output_directory': LaunchConfiguration('output'),
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
