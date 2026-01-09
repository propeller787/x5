from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('postback')
    param_file = os.path.join(pkg_share, 'config', 'event_alarm.yaml')

    return LaunchDescription([
        Node(
            package='postback',
            executable='event_alarm_node',
            name='event_alarm_node',
            parameters=[param_file],
            output='screen'
        )
    ])
