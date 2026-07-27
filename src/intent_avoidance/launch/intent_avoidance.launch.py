import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('intent_avoidance'),
        'config'
    )
    params_file = os.path.join(config_dir, 'intent_avoidance_params.yaml')

    return LaunchDescription([
        Node(
            package='intent_avoidance',
            executable='intent_avoidance_node',
            name='intent_avoidance_node',
            parameters=[params_file],
            output='screen',
        ),
    ])
