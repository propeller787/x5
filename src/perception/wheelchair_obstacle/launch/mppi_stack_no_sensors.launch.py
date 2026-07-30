# Bring up the whole MPPI navigation stack EXCEPT the sensor drivers
# (IMU / lidar drivers are assumed to be already running, or provided from a
# bag / external launch). This mirrors run_mppi_stack.sh but as a single launch
# so it does not depend on the lidar driver packages that were removed from the
# tree.
#
# Layers started here (toggle each with the matching argument):
#   perception   : horizontal_lidar_fusion -> obstacle_cloud_filter
#                  (+ localmap_occupymap when use_localmap_costmap:=true)
#   localization : odom_fusion_node (/odom/fused)
#   control      : controller_server (MPPI) + lifecycle_manager + joystick_local_path
#                  + cmd_vel_safety_gate  (via mppi_wheelchair_controller.launch.py)
#
# Costmap sources (use_localmap_costmap):
#   true  : occupymap_node + ExternalOccupymapLayer (mppi_controller_server_localmap.yaml)
#   false : RebuildObstacleLayer + InflationLayer (mppi_controller_server.yaml)
#
# Expected external inputs (from the skipped sensor / lidar layer):
#   /scan/left /scan/right /scan/back   (horizontal lidars)
#   /sensor/imu/data                    (IMU)
#   /wheel/twist                        (chassis controller)
#   robot TF: base_link->laser_frame_* and base_link->imu_link (from
#             robot_state_publisher / URDF, started alongside the lidar layer)
#
# Set start_rviz:=true to also open the navigation debug view.
# Set start_bag_record:=true to start cyclic rosbag2 recording (1 bag / interval,
# keep bag_record_max_count newest segments under bag_record_output_dir).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _include(package, launch_file, condition, launch_arguments=None):
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', launch_file)),
        condition=IfCondition(condition),
        launch_arguments=(launch_arguments or {}).items(),
    )


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_perception = LaunchConfiguration('start_perception')
    start_localization = LaunchConfiguration('start_localization')
    start_control = LaunchConfiguration('start_control')
    enable_safety_gate = LaunchConfiguration('enable_safety_gate')
    start_rviz = LaunchConfiguration('start_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    start_bag_record = LaunchConfiguration('start_bag_record')
    bag_record_interval_sec = LaunchConfiguration('bag_record_interval_sec')
    bag_record_max_count = LaunchConfiguration('bag_record_max_count')
    bag_record_output_dir = LaunchConfiguration('bag_record_output_dir')
    occupymap_params_file = LaunchConfiguration('occupymap_params_file')

    pkg_share = get_package_share_directory('wheelchair_obstacle')
    default_controller_params = os.path.join(
        pkg_share, 'config', 'mppi_controller_server_localmap.yaml')
    default_legacy_controller_params = os.path.join(
        pkg_share, 'config', 'mppi_controller_server.yaml')
    localization_params = os.path.join(
        get_package_share_directory('localization'), 'config', 'odom_fusion_params.yaml')
    bag_record_script = os.path.join(pkg_share, 'scripts', 'cyclic_rosbag_record.sh')
    default_bag_qos = os.path.join(pkg_share, 'config', 'mppi_stack_bag_qos.yaml')
    default_bag_topics = os.path.join(pkg_share, 'config', 'mppi_stack_bag_topics.txt')

    use_local = LaunchConfiguration('use_localmap_costmap').perform(context).lower() in (
        'true', '1', 'yes')
    params_override = LaunchConfiguration('controller_params_file').perform(context)
    if params_override:
        controller_params = params_override
    else:
        controller_params = (
            default_controller_params if use_local else default_legacy_controller_params)

    actions = [
        _include(
            'lidar_fusion', 'horizontal_lidar_fusion.launch.py', start_perception,
            {'use_sim_time': use_sim_time}),
        _include(
            'wheelchair_obstacle', 'obstacle_cloud_filter.launch.py', start_perception,
            {'use_sim_time': use_sim_time}),
    ]

    if use_local:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('localmap_occupymap'),
                'launch', 'occupymap.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': occupymap_params_file,
            }.items(),
        ))

    actions.extend([
        Node(
            package='localization',
            executable='odom_fusion_node',
            name='odom_fusion_node',
            output='screen',
            parameters=[localization_params, {'use_sim_time': use_sim_time}],
            condition=IfCondition(PythonExpression([
                "'", start_localization, "' == 'true' or '",
                start_localization, "' == '1' or '",
                start_localization, "' == 'yes'"
            ])),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                pkg_share, 'launch', 'mppi_wheelchair_controller.launch.py')),
            condition=IfCondition(start_control),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'enable_safety_gate': enable_safety_gate,
                'params_file': controller_params,
            }.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(start_rviz),
        ),
        ExecuteProcess(
            cmd=[
                bag_record_script,
                '--interval', bag_record_interval_sec,
                '--max-count', bag_record_max_count,
                '--output-dir', bag_record_output_dir,
                '--qos-file', default_bag_qos,
                '--topics-file', default_bag_topics,
            ],
            output='screen',
            condition=IfCondition(start_bag_record),
        ),
    ])

    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('wheelchair_obstacle')
    default_occupymap_params = os.path.join(
        pkg_share, 'config', 'occupymap_wheelchair.param.yaml')

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
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'nav_debug.rviz'),
            description='RViz2 config file for the navigation stack'),
        DeclareLaunchArgument(
            'start_bag_record', default_value='false',
            description='Start cyclic rosbag2 recording alongside the stack'),
        DeclareLaunchArgument(
            'bag_record_interval_sec', default_value='60',
            description='Seconds per rosbag segment (default 60 = 1 minute)'),
        DeclareLaunchArgument(
            'bag_record_max_count', default_value='10',
            description='Max rosbag segments to keep; oldest deleted when exceeded'),
        DeclareLaunchArgument(
            'bag_record_output_dir',
            default_value=os.path.join(os.path.expanduser('~'), 'bags', 'mppi_stack_cyclic'),
            description='Directory for cyclic rosbag segments (bag_YYYYMMDD_HHMMSS/)'),
        DeclareLaunchArgument(
            'use_localmap_costmap', default_value='true',
            description='Use localmap_occupymap + ExternalOccupymapLayer for MPPI costmap'),
        DeclareLaunchArgument(
            'controller_params_file', default_value='',
            description='Override controller_server params yaml (empty = auto by use_localmap_costmap)'),
        DeclareLaunchArgument(
            'occupymap_params_file', default_value=default_occupymap_params,
            description='occupymap_node params when use_localmap_costmap is true'),
        OpaqueFunction(function=_launch_setup),
    ])
