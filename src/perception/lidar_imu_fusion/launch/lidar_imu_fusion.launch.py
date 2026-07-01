from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("lidar_imu_fusion"),
        "config",
        "lidar_imu_fusion.yaml",
    ])

    return LaunchDescription([
        Node(
            package="lidar_imu_fusion",
            executable="three_lidar_imu_fusion_node",
            name="three_lidar_imu_fusion",
            output="screen",
            parameters=[params_file],
        )
    ])
