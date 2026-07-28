from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('localmap_occupymap'),
        'config',
        'occupymap.param.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=config_file,
            description='Path to occupymap parameter file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time from /clock',
        ),
        Node(
            package='localmap_occupymap',
            executable='occupymap_node',
            name='occupymap_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
