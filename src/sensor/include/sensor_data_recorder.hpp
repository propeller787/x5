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
#include <interface/srv/start_recording_with_path.hpp>
#include <interface/msg/vehicle_bind_status.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <utils/topic.h>
#include <filesystem>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <string>
#include <memory>
#include <chrono>
#include <vector>
#include <deque>
#include <thread>
#include <fstream>

namespace sensor::recorder {

class SensorDataRecorder : public rclcpp::Node {
public:
    SensorDataRecorder(const rclcpp::NodeOptions& options);
    ~SensorDataRecorder();

private:
    // 话题回调函数
    // cameraCallback 已移除，H264 文件直接在 h264_publisher 中落盘
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
    void startRecording(const std::string& folder_path);
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

    // 设备绑定回调函数
    void onDeviceBindStatus(const interface::msg::VehicleBindStatus::SharedPtr msg);
    
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
    bool containsSPSPPS(const std::vector<uint8_t>& h264_data);
    void cacheSPSPPS(const std::vector<uint8_t>& h264_data);
    std::vector<uint8_t> insertSPSPPSBeforeIDR(const std::vector<uint8_t>& h264_data);

    // 加载配置文件
    bool load();
    bool ensureDirExists(const std::string& path);

    /** 删除 uri 目录内 rosbag2 sqlite3 残留（metadata.yaml、*.db3），保留 H264 等其它文件；避免同一会话目录重复 open 报错 */
    void removeRosbag2SqliteArtifactsAtUri(const std::string& uri);

    /** IMU/GPS：已绑定（status 1 或 2）且（未分项下发 privacy 串，或串中含 location，大小写不敏感） */
    bool policyAllowsLocationTopics() const;
    /** CAN/雷达：已绑定且（未分项下发 privacy 串，或串中含 assist，大小写不敏感） */
    bool policyAllowsAssistTopics() const;

private:
    mutable std::mutex mutex_;
    // 订阅器
    // camera_sub_ 已移除，H264 文件直接在 h264_publisher 中落盘
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
    
    // 雷达数据订阅器
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_right_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_left_tilt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_right_tilt_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_back_sub_;

    // 设备绑定
    rclcpp::Subscription<interface::msg::VehicleBindStatus>::SharedPtr device_bind_status_sub_;
    
    // rosbag2 writer
    std::unique_ptr<rosbag2_cpp::Writer> bag_writer_;
    std::mutex writer_mutex_;
    std::atomic<bool> is_recording_{false};
    
    // 雷达数据独立的rosbag2 writer
    std::unique_ptr<rosbag2_cpp::Writer> lidar_bag_writer_;
    std::mutex lidar_writer_mutex_;
    std::atomic<bool> is_lidar_recording_{false};

    // 异步写盘队列与工作线程：回调只做序列化 + 入队，写盘放在独立线程，
    // 避免 SQLite 写盘阻塞 ROS2 executor 线程导致 chassis 心跳定时器抖动/丢失。
    struct PendingBagMessage {
        std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_msg;
    };
    static constexpr size_t kMaxPendingQueueSize = 5000;  // 反压：超过则丢弃最旧消息

    std::deque<PendingBagMessage> sensor_queue_;
    std::mutex sensor_queue_mutex_;
    std::condition_variable sensor_queue_cv_;
    std::thread sensor_writer_thread_;
    std::atomic<uint64_t> sensor_dropped_msgs_{0};

    std::deque<PendingBagMessage> lidar_queue_;
    std::mutex lidar_queue_mutex_;
    std::condition_variable lidar_queue_cv_;
    std::thread lidar_writer_thread_;
    std::atomic<uint64_t> lidar_dropped_msgs_{0};

    std::atomic<bool> writer_threads_stop_{false};
    std::atomic<bool> sensor_writer_restarting_{false};
    std::atomic<bool> lidar_writer_restarting_{false};

    void sensorWriterLoop();
    void lidarWriterLoop();
    void enqueueSensorMessage(PendingBagMessage&& msg);
    void enqueueLidarMessage(PendingBagMessage&& msg);

    // 后台清理线程（替代 ROS timer）：避免在 executor 中执行
    // recursive_directory_iterator 这种长耗时 IO 操作。
    std::thread cleanup_thread_;
    std::mutex cleanup_cv_mutex_;
    std::condition_variable cleanup_cv_;
    std::atomic<bool> cleanup_stop_{false};
    void cleanupLoop();
    
    // 序列化器
    // camera_serialization_ 已移除，H264 文件直接在 h264_publisher 中落盘
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serialization_;
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> gps_serialization_;
    rclcpp::Serialization<can_msgs::msg::Frame> can_serialization_;
    rclcpp::Serialization<sensor_msgs::msg::LaserScan> lidar_serialization_;
    
    // 配置参数
    std::string record_dir_;
    std::string bag_prefix_;
    std::string lidar_bag_prefix_;  // 雷达数据bag文件前缀
    int max_bag_duration_minutes_;  // sensor数据每个bag文件的最大时长（分钟）
    int max_bag_size_mb_;           // sensor数据每个bag文件的最大大小（MB）
    int max_lidar_bag_duration_minutes_;  // lidar数据每个bag文件的最大时长（分钟）
    int max_lidar_bag_size_mb_;           // lidar数据每个bag文件的最大大小（MB）
    int retention_minutes_;         // 环形缓冲区保留时长（分钟），默认30分钟
    bool auto_start_recording_;     // 启动时自动开始录制
    bool auto_start_lidar_recording_;  // 启动时自动开始雷达数据录制
    
    // 验证模式配置参数
    bool verify_camera_enable_;     // 是否启用camera数据验证（解码H264为图片）
    std::string verify_image_dir_;  // 验证图片保存目录
    int verify_frame_interval_;     // 验证模式下保存图片的帧间隔（每N帧保存一张）
    std::vector<uint8_t> h264_buffer_;  // H264数据缓冲区（用于累积数据）
    std::mutex verify_mutex_;        // 验证模式互斥锁
    uint32_t verify_frame_count_;    // 验证模式帧计数器

    // 隐私协议
    std::string privacy_policy_ = "";
    /** VehicleBindStatus.status：0 未绑定 1 已绑定 2 隐私变更（本地 YAML 会与 TFT 约定互转后再与此对齐） */
    int app_bind_page_state_{0};
    
    // H.264 SPS/PPS 缓存（用于确保 GOP 完整性）
    std::vector<uint8_t> cached_sps_;  // 缓存的 SPS 参数集
    std::vector<uint8_t> cached_pps_;  // 缓存的 PPS 参数集
    bool has_cached_sps_pps_;          // 是否已缓存 SPS/PPS
    std::mutex sps_pps_mutex_;         // SPS/PPS 缓存互斥锁
    
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
    
    // 文件清理：现已迁移到 cleanup_thread_，不再使用 wall_timer。
    // cleanupOldFiles 仍然保留，由 cleanupLoop 周期性调用。
    void cleanupOldFiles();
    
    // 参数回调句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    
    // 服务 - 控制录制开始/停止
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_record_service_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_record_service_;
    rclcpp::Service<interface::srv::StartRecordingWithPath>::SharedPtr start_record_with_path_service_;
    
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
    
    void startRecordWithPathServiceCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<interface::srv::StartRecordingWithPath::Request> request,
        std::shared_ptr<interface::srv::StartRecordingWithPath::Response> response);
};

} // namespace sensor::recorder

