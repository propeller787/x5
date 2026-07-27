from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("shared_control_assist")
    params = os.path.join(pkg_dir, "config", "shared_control_assist.yaml")

    return LaunchDescription([
        Node(
            package="shared_control_assist",
            executable="shared_control_assist_node",
            name="shared_control_assist",
            output="screen",
            parameters=[params],
        )
    ])
