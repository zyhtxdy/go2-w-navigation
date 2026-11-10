import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('semantic_navigator')
    params_file = LaunchConfiguration('params_file')
    semantic_map_file = LaunchConfiguration('semantic_map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                package_share, 'config', 'semantic_navigator.yaml'
            ),
            description='Semantic navigator ROS parameter file.',
        ),
        DeclareLaunchArgument(
            'semantic_map_file',
            default_value=os.path.join(
                package_share, 'config', 'semantic_locations.yaml'
            ),
            description='Semantic entity and candidate-pose map.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        Node(
            package='semantic_navigator',
            executable='semantic_navigator',
            name='semantic_navigator',
            output='screen',
            parameters=[
                params_file,
                {
                    'semantic_map_file': semantic_map_file,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
    ])
