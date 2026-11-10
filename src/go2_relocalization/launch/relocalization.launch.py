import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('go2_relocalization')
    return LaunchDescription([
        DeclareLaunchArgument('database', description='Saved keyframe database directory.'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'relocalization.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('publish_known_map', default_value='false'),
        DeclareLaunchArgument('candidate_threads', default_value='1'),
        # TF is deliberately not a launch option for the observation prototype.
        Node(package='go2_relocalization', executable='relocalization_node',
             name='relocalization', output='screen', parameters=[
                 LaunchConfiguration('params_file'), {
                     'database_directory': LaunchConfiguration('database'),
                     'publish_tf': False,
                     'registration.candidate_threads': ParameterValue(
                         LaunchConfiguration('candidate_threads'), value_type=int),
                     'publish_known_map': ParameterValue(
                         LaunchConfiguration('publish_known_map'), value_type=bool),
                     'use_sim_time': ParameterValue(
                         LaunchConfiguration('use_sim_time'), value_type=bool),
                 }]),
    ])
