from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cloud_manager',
            executable='oss_log_uploader_node',
            name='oss_log_uploader_node',
            output='screen',
            parameters=[
                # 预留阿里云参数，后续直接替换为真实值
                {'oss.endpoint': 'oss-cn-hangzhou.aliyuncs.com'},
                {'oss.access_key_id': 'mock_ak'},
                {'oss.access_key_secret': 'mock_sk'},
                {'oss.bucket_name': 'mock_bucket'},
                {'oss.device_model': 'x5'},
                {'oss.device_id': 'X001'},
                {'oss.log_base_path': '/tmp/vehicle_logs/'}
            ]
        )
    ])