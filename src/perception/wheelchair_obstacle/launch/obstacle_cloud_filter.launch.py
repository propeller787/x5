import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wheelchair_obstacle')
    params_file = os.path.join(package_share, 'config', 'obstacle_cloud_filter.yaml')

    return LaunchDescription([
        Node(
            package='wheelchair_obstacle',
            executable='obstacle_cloud_filter_node',
            name='obstacle_cloud_filter',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file],
            arguments=['--ros-args', '--log-level', 'INFO'],
        )
    ])
