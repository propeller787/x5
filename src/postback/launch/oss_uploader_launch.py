from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='postback',  # 修正为postback包名（与package.xml一致）
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
                {'oss.log_base_path': '/tmp/vehicle_logs/'},
                # 补充节点核心参数默认值（可选，增强鲁棒性）
                {'device_model': 'x5'},
                {'device_id': 'X001'},
                {'upload_interval_seconds': 1800}
            ]
        )
    ])