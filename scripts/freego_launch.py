from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch.conditions import IfCondition

def generate_launch_description():
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'start_sensor',
            default_value='true',
            description='Enable/disable sensor node'
        ),
        DeclareLaunchArgument(
            'start_rosbridge',
            default_value='true',
            description='Enable/disable rosbridge'
        ),
        DeclareLaunchArgument(
            'start_lidar',
            default_value='true',
            description='Enable/disable lidar node'
        ),
        DeclareLaunchArgument(
            'start_camera',
            default_value='false',
            description='Enable/disable camera node'
        ),
        DeclareLaunchArgument(
            'start_control_service',
            default_value='true',
            description='Enable/disable control service node'
        ), 
        DeclareLaunchArgument(
            'start_ahrs',
            default_value='true',
            description='Enable/disable ahrs node'
        ),
        DeclareLaunchArgument(
            'start_perception_control',
            default_value='true',
            description='Enable/disable perception control node'
        ),
        DeclareLaunchArgument(
            'start_scene_auxiliary',
            default_value='true',
            description='Enable/disable scene auxiliary node'
        ),
        DeclareLaunchArgument(
            'start_speed_bump',
            default_value='true',
            description='Enable/disable speed bump node'
        ),
        DeclareLaunchArgument(
            'start_po_dao',
            default_value='true',
            description='Enable/disable po dao node'
        ),
        DeclareLaunchArgument(
            'start_scene_dance',
            default_value='true',
            description='Enable/disable scene dance node'
        ),
        DeclareLaunchArgument(
            'start_scene_service',
            default_value='true',
            description='Enable/disable scene service node'
        ),
        DeclareLaunchArgument(
            'start_lidar_obstacle',
            default_value='true',
            description='Enable/disable lidar obstacle node'
        ),
        DeclareLaunchArgument(
            'start_follow_me',
            default_value='true',
            description='Enable/disable follow me node'
        ),
        DeclareLaunchArgument(
            'start_ydlidar',
            default_value='false',
            description='Enable/disable ydlidar Tmini node'
        ),
        # Sensor节点（条件启动）
        Node(
            package='sensor',
            namespace='sensor',
            executable='sensor',
            name='sensor',
            respawn=False,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_sensor'))
        ),
        Node(
            package='control_service',
            executable='control_service',
            name='control_service',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_control_service'))
        ),  
        # Rosbridge节点（条件启动）
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[
                {'port': 9090,
                'max_message_size': 1024 * 1024 * 5,
                'allow_topics': ['/vlago/recharge/status', '/vlago/alarm', 
                        '/vlago/mode/status', '/vlago/auxiliary/alarm', 
                        '/vlago/backcar/alarm','/sensor/backup/car',
                        '/sensor/backup/camera/h264','/xhi/status',
                        '/vlago/car/info','/scene_services/mode','/speed_bump/status',
                        '/downhill/status','/vlago/cruise/status'],
                'allow_services': ['/vlago/gear', '/vlago/auto_charge', 
                        '/vlago/car_id', '/vlago/setting', '/vlago/mode/enable',
                         '/vlago/narrow_assistant_mode', '/vlago/safe_mode',
                         '/vlago/standard_mode','/vlago/dance_show','/manage/cruise'],
                'allow_actions': []
        
                }
            ],
            condition=IfCondition(LaunchConfiguration('start_rosbridge'))
        ),
        # LiDAR节点（条件启动）
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='MS200_right',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {'lidar1_device_model': 'MS200'},
                {'lidar1_frame_id': 'laser_frame_right'},
                {'lidar1_scan_topic': 'scan/right'},
                {'lidar1_port_name': '/dev/ttyS4'},
                {'lidar1_baudrate': 230400},
                {'lidar1_angle_min': 200.0},
                {'lidar1_angle_max': 360.0},
                # {'lidar1_angle_ranges': [0.0,31.0,250.0,360.0]},
                {'lidar1_angle_ranges': [0.0,22.0,250.0,360.0]},
                {'lidar1_range_min': 0.05},
                {'lidar1_range_max': 10.0},
                {'lidar1_clockwise': False},
                {'lidar1_motor_speed': 10}
            ],
            condition=IfCondition(LaunchConfiguration('start_lidar'))
        ),
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='MS200_left',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {'lidar1_device_model': 'MS200'},
                {'lidar1_frame_id': 'laser_frame_left'},
                {'lidar1_scan_topic': 'scan/left'},
                {'lidar1_port_name': '/dev/ttyS0'},
                {'lidar1_baudrate': 230400},
                {'lidar1_angle_min': 0.0},
                {'lidar1_angle_max': 160.0},
                # {'lidar1_angle_ranges': [0.0,110.0,329.0,360.0]},
                {'lidar1_angle_ranges': [0.0,110.0,338.0,360.0]},
                {'lidar1_range_min': 0.05},
                {'lidar1_range_max': 10.0},
                {'lidar1_clockwise': False},
                {'lidar1_motor_speed': 10}
            ],
            condition=IfCondition(LaunchConfiguration('start_lidar'))
        ),
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='MS200_left_tilt',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {'lidar1_device_model': 'MS200'},
                {'lidar1_frame_id': 'laser_frame_left_tilt'},
                {'lidar1_scan_topic': 'scan/left_tilt'},
                {'lidar1_port_name': '/dev/ttyS7'},
                {'lidar1_baudrate': 230400},
                {'lidar1_angle_min': 0.0},
                {'lidar1_angle_max': 360.0},
                {'lidar1_angle_ranges': [0.0,80.0,285.0,360.0]},
                {'lidar1_range_min': 0.05},
                {'lidar1_range_max': 10.0},
                {'lidar1_clockwise': False},
                {'lidar1_motor_speed': 10}
            ],
            condition=IfCondition(LaunchConfiguration('start_lidar'))
        ),
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='MS200_right_tilt',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {'lidar1_device_model': 'MS200'},
                {'lidar1_frame_id': 'laser_frame_right_tilt'},
                {'lidar1_scan_topic': 'scan/right_tilt'},
                {'lidar1_port_name': '/dev/ttyS8'},
                {'lidar1_baudrate': 230400},
                {'lidar1_angle_min': 0.0},
                {'lidar1_angle_max': 360.0},
                {'lidar1_angle_ranges': [0.0,75.0,280.0,360.0]},
                {'lidar1_range_min': 0.05},
                {'lidar1_range_max': 10.0},
                {'lidar1_clockwise': False},
                {'lidar1_motor_speed': 10}
            ],
            condition=IfCondition(LaunchConfiguration('start_lidar'))
        ),
        Node(
            package='oradar_lidar',
            executable='oradar_scan',
            name='MS200_back',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[
                {'lidar1_device_model': 'MS200'},
                {'lidar1_frame_id': 'laser_frame_back'},
                {'lidar1_scan_topic': 'scan/back'},
                {'lidar1_port_name': '/dev/ttyS9'},
                {'lidar1_baudrate': 230400},
                {'lidar1_angle_min': 0.0},
                {'lidar1_angle_max': 360.0},
                {'lidar1_angle_ranges': [0.0, 35.0, 116, 210, 220, 320]},
                {'lidar1_range_min': 0.05},
                {'lidar1_range_max': 10.0},
                {'lidar1_clockwise': False},
                {'lidar1_motor_speed': 10}
            ],
            condition=IfCondition(LaunchConfiguration('start_lidar'))
        ),
        # YDLidar Tmini 节点（条件启动）
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[{
                # 串口设备路径
                'port': '/dev/ttyS1',
                # ROS 坐标系名称
                'frame_id': 'laser_frame',
                # 忽略角度数组（空字符串表示不忽略任何角度）(例如: "0,30;330,360")
                'ignore_array': '',
                # 串口波特率
                'baudrate': 230400,
                # 激光雷达类型标识符（1 表示 Tmini 系列）
                'lidar_type': 1,
                # 设备类型标识符（0 表示默认设备类型）
                'device_type': 0,
                # 是否启用强度信息
                'intensity': True,
                # 强度数据的位数（8 位）
                'intensity_bit': 8,
                # 是否为单通道模式（false: 双通道模式）
                'isSingleChannel': False,
                # 扫描频率，单位：Hz
                'frequency': 12.0,
                # 采样率，控制扫描点的密度
                'sample_rate': 4,
                # 异常检测计数阈值
                'abnormal_check_count': 4,
                # 是否使用固定分辨率
                'fixed_resolution': True,
                # 是否启用调试模式
                'debug': False,
            }],
            namespace='/',
            condition=IfCondition(LaunchConfiguration('start_ydlidar'))
        ),
        Node(
            package='perception_control',
            executable='perception_control_node',
            name='perception_control_node',
            namespace='perception_control',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_perception_control'))
        ),
        Node(
            package='scene_auxiliary',
            executable='scene_auxiliary_node',
            name='scene_auxiliary_node',
            namespace='scene_auxiliary',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_scene_auxiliary'))
        ),
        Node(
            package='speedbump_detect',
            executable='speedbump_detect_node',
            name='speedbump_detect_node',
            #namespace='speedbump',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_speed_bump'))
        ),
        Node(
            package='podao_detect',
            executable='podao_detect_node',
            name='podao_detect_node',
            #namespace='podao_detect',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_po_dao'))
        ),
        Node(
            package='lidar_obstacle_groundmodel',
            executable='lidar_obstacle_groundmodel_node',
            name='lidar_obstacle_groundmodel',
            # namespace='lidar_obstacle',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_lidar'))
        ),
        Node(
            package='scene_dance',
            executable='scene_dance_node',
            name='scene_dance_node',
            namespace='scene_dance',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'WARN'],
            condition=IfCondition(LaunchConfiguration('start_scene_dance'))
        ),
        Node(
            package='scene_service',
            executable='scene_service',
            name='scene_service',
            namespace='scene_service',
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'INFO'],
            condition=IfCondition(LaunchConfiguration('start_scene_service'))
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            arguments=['/home/opt/freego/scripts/yisbot_o1.urdf'],
            condition=IfCondition(LaunchConfiguration('start_lidar'))
        )
    ])
