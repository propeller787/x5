import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('lidar_fusion')
    params_file = os.path.join(package_share, 'config', 'horizontal_lidar_fusion.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        Node(
            package='lidar_fusion',
            executable='horizontal_lidar_fusion_node',
            name='horizontal_lidar_fusion',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', 'WARN'],
        )
    ])
