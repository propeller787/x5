#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
parameters=[
        {'device_model': 'MS200'},
        {'frame_id': 'laser_frame'},
        {'scan_topic': 'MS200/scan'},
        {'port_name': '/dev/ttyACM0'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 360.0},
        {'range_min': 0.05},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
'''

"""
此代码段定义了一个名为 `generate_launch_description` 的函数，用于生成 ROS2 启动描述。
该函数创建了两个节点：
1. `ordlidar_node` 是 LiDAR 发布节点，配置了设备型号、帧 ID、扫描主题、端口名称、波特率、角度范围、距离范围、旋转方向和电机速度等参数。
2. `base_link_to_laser_tf_node` 是从 base_link 到 laser_frame 的 tf 节点，用于发布坐标变换信息。
最后，将这两个节点添加到 `LaunchDescription` 对象中并返回。
雷达正后方为0度  正前方为180度
"""
def generate_launch_description():
  # LiDAR publisher node
  ordlidar_node = Node(
      package='oradar_lidar',
      executable='oradar_scan',
      name='MS200',
      output='screen',
      parameters=[
        {'lidar1_device_model': 'MS2001'},  # LiDAR 1 设备型号
        {'lidar1_frame_id': 'laser_frame_right'},  # LiDAR 1 帧 ID
        {'lidar1_scan_topic': 'scan/right'},  # LiDAR 1 扫描主题
        {'lidar1_port_name': '/dev/ttyS2'},  # LiDAR 1 端口名称
        {'lidar1_baudrate': 230400},  # LiDAR 1 波特率
        {'lidar1_angle_ranges': [0.0, 160.0]},
        {'lidar1_range_min': 0.05},  # LiDAR 1 最小距离
        {'lidar1_range_max': 10.0},  # LiDAR 1 最大距离
        {'lidar1_clockwise': False},  # LiDAR 1 旋转方向
        {'lidar1_motor_speed': 10},  # LiDAR 1 电机速度
      ]
  )

  # base_link to laser_frame tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser',
    arguments=['0','0','0.18','0','0','0','base_link','laser_frame']
  )
  # Define LaunchDescription variable
  ord = LaunchDescription()
  ord.add_action(ordlidar_node)
  #ord.add_action(base_link_to_laser_tf_node)

  return ord