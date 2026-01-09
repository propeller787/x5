from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    broker_arg = DeclareLaunchArgument(
        "mqtt_broker",
        default_value="mqtts://39.106.140.222:28883",
        description="MQTT broker URL"
    )
    client_id_arg = DeclareLaunchArgument(
        "mqtt_client_id",
        default_value="cloud_mgr_test",
        description="MQTT client id"
    )
    username_arg = DeclareLaunchArgument(
        "mqtt_username",
        default_value="token_user",
        description="MQTT username"
    )
    password_arg = DeclareLaunchArgument(
        "mqtt_password",
        default_value="",
        description="MQTT password/JWT"
    )
    log_pre_arg = DeclareLaunchArgument(
        "log_pre_publish",
        default_value="true",
        description="Print pre-publish payload values"
    )
    log_mileage_arg = DeclareLaunchArgument(
        "log_mileage",
        default_value="true",
        description="Enable mileage calculator logs"
    )
    gps_sim_arg = DeclareLaunchArgument(
        "gps_simulate",
        default_value="false",
        description="Enable GPS simulation when no real source"
    )

    node = Node(
        package="cloud_manager",
        executable="cloud_manager_node",
        name="cloud_manager_node",
        output="screen",
        parameters=[
            {
                "mqtt.broker": LaunchConfiguration("mqtt_broker"),
                "mqtt.client_id": LaunchConfiguration("mqtt_client_id"),
                "mqtt.username": LaunchConfiguration("mqtt_username"),
                "mqtt.password": LaunchConfiguration("mqtt_password"),
                "mqtt.insecure_skip_verify": True,
                "log.pre_publish": LaunchConfiguration("log_pre_publish"),
                "log.mileage": LaunchConfiguration("log_mileage"),
                "gps.simulate": LaunchConfiguration("gps_simulate"),
            }
        ]
    )

    return LaunchDescription([
        broker_arg,
        client_id_arg,
        username_arg,
        password_arg,
        log_pre_arg,
        log_mileage_arg,
        gps_sim_arg,
        node,
    ])