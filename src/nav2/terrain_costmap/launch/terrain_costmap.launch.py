import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('terrain_costmap')
    return LaunchDescription([
        Node(
            package='terrain_costmap',
            executable='terrain_costmap_node',
            name='terrain_costmap',
            output='screen',
            parameters=[
                os.path.join(share, 'config', 'terrain_costmap.yaml'),
            ],
        ),
    ])
