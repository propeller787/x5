# Bring up the whole MPPI navigation stack EXCEPT the sensor drivers
# (IMU / lidar drivers are assumed to be already running, or provided from a
# bag / external launch). This mirrors run_mppi_stack.sh but as a single launch
# so it does not depend on the lidar driver packages that were removed from the
# tree.
#
# Layers started here (toggle each with the matching argument):
#   perception   : horizontal_lidar_fusion -> obstacle_cloud_filter
#                  (the local costmap is now built inside controller_server via
#                   wheelchair_obstacle::RebuildObstacleLayer — a per-cycle
#                   full-rebuild ObstacleLayer variant — + InflationLayer)
#   localization : odom_fusion_node (/odom/fused)
#   control      : controller_server (MPPI) + lifecycle_manager + joystick_local_path
#                  + cmd_vel_safety_gate  (via mppi_wheelchair_controller.launch.py)
#
# Expected external inputs (from the skipped sensor / lidar layer):
#   /scan/left /scan/right /scan/back   (horizontal lidars)
#   /sensor/imu/data                    (IMU)
#   /wheel/twist                        (chassis controller)
#   robot TF: base_link->laser_frame_* and base_link->imu_link (from
#             robot_state_publisher / URDF, started alongside the lidar layer)
#
# Set start_rviz:=true to also open the navigation debug view.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package, launch_file, condition, launch_arguments=None):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file)),
        condition=IfCondition(condition),
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_perception = LaunchConfiguration('start_perception')
    start_localization = LaunchConfiguration('start_localization')
    start_control = LaunchConfiguration('start_control')
    enable_safety_gate = LaunchConfiguration('enable_safety_gate')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    pkg_share = get_package_share_directory('wheelchair_obstacle')
    localization_params = os.path.join(
        get_package_share_directory('localization'), 'config', 'odom_fusion_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'nav_debug.rviz')

    # --- perception layer ---
    lidar_fusion = _include(
        'lidar_fusion', 'horizontal_lidar_fusion.launch.py', start_perception)
    obstacle_filter = _include(
        'wheelchair_obstacle', 'obstacle_cloud_filter.launch.py', start_perception)

    # --- localization layer (no launch file in the package -> run the node) ---
    odom_fusion = Node(
        package='localization',
        executable='odom_fusion_node',
        name='odom_fusion_node',
        output='screen',
        parameters=[localization_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_localization),
    )

    # --- control layer ---
    control = _include(
        'wheelchair_obstacle', 'mppi_wheelchair_controller.launch.py', start_control,
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_safety_gate': enable_safety_gate,
        })

    # --- debug visualization (optional) ---
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(start_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock if true'),
        DeclareLaunchArgument(
            'start_perception', default_value='true',
            description='Start lidar fusion + obstacle cloud filter'),
        DeclareLaunchArgument(
            'start_localization', default_value='true',
            description='Start odom_fusion_node (/odom/fused)'),
        DeclareLaunchArgument(
            'start_control', default_value='true',
            description='Start MPPI controller_server + joystick + safety gate'),
        DeclareLaunchArgument(
            'enable_safety_gate', default_value='true',
            description='Launch cmd_vel_safety_gate_node between /cmd_vel and /nav/control'),
        DeclareLaunchArgument(
            'start_rviz', default_value='false',
            description='Open RViz2 with the navigation debug config'),
        DeclareLaunchArgument(
            'rviz_config', default_value=default_rviz,
            description='RViz2 config file for the navigation stack'),
        lidar_fusion,
        obstacle_filter,
        odom_fusion,
        control,
        rviz,
    ])
