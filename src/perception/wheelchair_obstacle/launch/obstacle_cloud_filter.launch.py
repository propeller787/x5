import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wheelchair_obstacle')
    params_file = os.path.join(package_share, 'config', 'obstacle_cloud_filter.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        Node(
            package='wheelchair_obstacle',
            executable='obstacle_cloud_filter_node',
            name='obstacle_cloud_filter',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'INFO'],
        )
    ])
