import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('wheelchair_obstacle')

    default_params = os.path.join(package_share, 'config', 'mppi_controller_server.yaml')
    default_joystick_params = os.path.join(package_share, 'config', 'joystick_local_path.yaml')
    default_safety_params = os.path.join(package_share, 'config', 'cmd_vel_safety_gate.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    joystick_params_file = LaunchConfiguration('joystick_params_file')
    safety_params_file = LaunchConfiguration('safety_params_file')
    enable_safety_gate = LaunchConfiguration('enable_safety_gate')

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odom', '/odom/fused'),
        ],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['controller_server'],
        }],
    )

    joystick_local_path = Node(
        package='wheelchair_obstacle',
        executable='joystick_local_path_node',
        name='joystick_local_path_node',
        output='screen',
        parameters=[joystick_params_file, {'use_sim_time': use_sim_time}],
    )

    cmd_vel_safety_gate = Node(
        package='wheelchair_obstacle',
        executable='cmd_vel_safety_gate_node',
        name='cmd_vel_safety_gate_node',
        output='screen',
        parameters=[safety_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(enable_safety_gate),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Nav2 controller_server + local_costmap params (plan 4.8)'),
        DeclareLaunchArgument(
            'joystick_params_file',
            default_value=default_joystick_params,
            description='joystick_local_path_node params'),
        DeclareLaunchArgument(
            'safety_params_file',
            default_value=default_safety_params,
            description='cmd_vel_safety_gate_node params'),
        DeclareLaunchArgument(
            'enable_safety_gate',
            default_value='true',
            description='Launch cmd_vel_safety_gate_node between /cmd_vel and /nav/control'),
        controller_server,
        lifecycle_manager,
        joystick_local_path,
        cmd_vel_safety_gate,
    ])
