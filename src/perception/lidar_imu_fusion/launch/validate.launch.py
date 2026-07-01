# =============================================================================
# validate.launch.py —— 一键启动融合节点 + 验证工具
#
# 用法:
#   # 默认验证 30s, 监控正前方(0°)稳定性
#   ros2 launch lidar_imu_fusion validate.launch.py
#
#   # 自定义参数
#   ros2 launch lidar_imu_fusion validate.launch.py duration:=60 watch_angle:=0.0
#
# 前置条件: /scan/left /scan/right /scan/back /sensor/imu/data 正在发布,
#           且 TF 树(base_link <- laser_frame_* / imu_link)可用或 YAML 静态外参已填。
# =============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare("lidar_imu_fusion"),
        "config",
        "lidar_imu_fusion.yaml",
    ])

    duration_arg = DeclareLaunchArgument(
        "duration", default_value="30.0",
        description="验证采集时长(秒), 0=持续运行"
    )
    watch_angle_arg = DeclareLaunchArgument(
        "watch_angle", default_value="0.0",
        description="监控稳定性的角度(度, 0=正前方)"
    )
    output_arg = DeclareLaunchArgument(
        "output", default_value="",
        description="报告写入的 JSON 文件路径, 留空则只打印不写文件"
    )

    fusion_node = Node(
        package="lidar_imu_fusion",
        executable="three_lidar_imu_fusion_node",
        name="three_lidar_imu_fusion",
        output="screen",
        parameters=[params_file],
    )

    # validator 作为独立进程, 用 ros2 run 调用已安装的脚本
    validator = ExecuteProcess(
        cmd=[
            "ros2", "run", "lidar_imu_fusion", "fusion_validator.py",
            "--duration", LaunchConfiguration("duration"),
            "--watch-angle", LaunchConfiguration("watch_angle"),
            "--output", LaunchConfiguration("output"),
        ],
        output="screen",
    )

    return LaunchDescription([
        duration_arg,
        watch_angle_arg,
        output_arg,
        fusion_node,
        validator,
    ])
