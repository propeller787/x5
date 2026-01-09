#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>
#include <can_msgs/msg/frame.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#include <std_srvs/srv/empty.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <utils/topic.h>
#include <filesystem>
#include <mutex>
#include <atomic>
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <fstream>

namespace sensor::recorder {

class SensorDataRecorder : public rclcpp::Node {
public:
    SensorDataRecorder(const rclcpp::NodeOptions& options);
    ~SensorDataRecorder();

private:
    // 话题回调函数
    void cameraCallback(const ffmpeg_image_transport_msgs::msg::FFMPEGPacket::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void canCallback(const can_msgs::msg::Frame::SharedPtr msg);
    
    // 雷达数据回调函数
    void lidarRightCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lidarLeftCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lidarLeftTiltCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lidarRightTiltCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void lidarBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // 录制相关函数
    void startRecording();
    void stopRecording();
    void writeMessage(const std::string& topic_name, const std::string& topic_type, 
                     const rclcpp::Time& timestamp, const void* msg_ptr);
    
    // 雷达数据独立录制相关函数
    void startLidarRecording();
    void stopLidarRecording();
    void writeLidarMessage(const std::string& topic_name, const std::string& topic_type,
                          const rclcpp::Time& timestamp, const void* msg_ptr);
    std::string generateLidarBagPath();
    void onLidarBagCheckTimer();
    
    // 配置管理
    void loadConfig();
    void createBagDirectory();
    std::string generateBagPath();
    
    // 参数回调
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters);
    
    // 验证模式相关函数
    void decodeAndSaveImage(const std::vector<uint8_t>& h264_data, const rclcpp::Time& timestamp);
    void createVerifyImageDirectory();
    std::string generateImagePath(const rclcpp::Time& timestamp);
    bool hasIDRFrame(const std::vector<uint8_t>& h264_data);
    bool hasValidH264StartCode(const std::vector<uint8_t>& h264_data);

private:
    // 订阅器
    rclcpp::Subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    
    // 雷达数据订阅器
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_left_tilt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_right_tilt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_back_sub_;
    
    // rosbag2 writer
    std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;
    std::mutex writer_mutex_;
    std::atomic<bool> is_recording_{false};
    
    // 雷达数据独立的rosbag2 writer
    std::unique_ptr<rosbag2_cpp::Writer> lidar_bag_writer_;
    std::mutex lidar_writer_mutex_;
    std::atomic<bool> is_lidar_recording_{false};
    
    // 序列化器
    rclcpp::Serialization<ffmpeg_image_transport_msgs::msg::FFMPEGPacket> camera_serialization_;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization_;
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> gps_serialization_;
    rclcpp::Serialization<can_msgs::msg::Frame> can_serialization_;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> lidar_serialization_;
    
    // 配置参数
    std::string record_dir_;
    std::string bag_prefix_;
    std::string lidar_bag_prefix_;  // 雷达数据bag文件前缀
    int max_bag_duration_minutes_;  // 每个bag文件的最大时长（分钟）
    int max_bag_size_mb_;           // 每个bag文件的最大大小（MB）
    bool auto_start_recording_;     // 启动时自动开始录制
    bool auto_start_lidar_recording_;  // 启动时自动开始雷达数据录制
    
    // 验证模式配置参数
    bool verify_camera_enable_;     // 是否启用camera数据验证（解码H264为图片）
    std::string verify_image_dir_;  // 验证图片保存目录
    int verify_frame_interval_;     // 验证模式下保存图片的帧间隔（每N帧保存一张）
    std::vector<uint8_t> h264_buffer_;  // H264数据缓冲区（用于累积数据）
    std::mutex verify_mutex_;        // 验证模式互斥锁
    uint32_t verify_frame_count_;    // 验证模式帧计数器
    
    // 当前bag文件信息
    std::string current_bag_path_;
    std::chrono::system_clock::time_point current_bag_start_time_;
    int64_t current_bag_size_bytes_;
    
    // 当前雷达bag文件信息
    std::string current_lidar_bag_path_;
    std::chrono::system_clock::time_point current_lidar_bag_start_time_;
    int64_t current_lidar_bag_size_bytes_;
    
    // 定时器 - 用于定期检查是否需要创建新的bag文件
    rclcpp::TimerBase::SharedPtr bag_check_timer_;
    void onBagCheckTimer();
    
    // 雷达数据定时器
    rclcpp::TimerBase::SharedPtr lidar_bag_check_timer_;
    
    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // 服务 - 控制录制开始/停止
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_record_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_record_service_;
    
    // 雷达数据录制服务
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_lidar_record_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_lidar_record_service_;
    
    void startRecordServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    void stopRecordServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    void startLidarRecordServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    
    void stopLidarRecordServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
};

} // namespace sensor::recorder

