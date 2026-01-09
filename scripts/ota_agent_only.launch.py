from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch 参数声明，便于在命令行覆盖
    args = [
        DeclareLaunchArgument('mqtt_broker', default_value='mqtts://39.106.140.222:28883'),
        DeclareLaunchArgument('mqtt_client_id', default_value='ota_agent_test'),
        DeclareLaunchArgument('mqtt_username', default_value='token_user'),
        DeclareLaunchArgument('mqtt_password', default_value=''),
        DeclareLaunchArgument('mqtt_insecure_skip_verify', default_value='true'),
        DeclareLaunchArgument('vin', default_value='VIN_TEST'),
        DeclareLaunchArgument('model', default_value='1'),
        DeclareLaunchArgument('model_str', default_value='default'),
        DeclareLaunchArgument('log_level', default_value='INFO'),
    ]

    node = Node(
        package='ota_agent',
        executable='ota_agent_node',
        name='ota_agent',
        output='screen',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        parameters=[
            {'mqtt.broker_url': LaunchConfiguration('mqtt_broker')},
            {'mqtt.client_id': LaunchConfiguration('mqtt_client_id')},
            {'mqtt.username': LaunchConfiguration('mqtt_username')},
            {'mqtt.password': LaunchConfiguration('mqtt_password')},
            {'mqtt.insecure_skip_verify': LaunchConfiguration('mqtt_insecure_skip_verify')},
            {'vin': LaunchConfiguration('vin')},
            {'model': LaunchConfiguration('model')},
            {'model_str': LaunchConfiguration('model_str')},
        ],
    )

    return LaunchDescription(args + [node])
