#include "sensor_data_recorder.hpp"
#include <std_srvs/srv/empty.hpp>
#include <interface/srv/start_recording_with_path.hpp>
#include <iomanip>
#include <sstream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>
#include <rcutils/allocator.h>

namespace sensor::recorder {
const char* kAppBindPageState = "app_bind_page_state";
const char* kPrivacyPolicy = "privacy_policy";

// 单次分配 rcutils_uint8_array_t + buffer（struct 后紧跟数据），一次 malloc / free
static std::shared_ptr<rcutils_uint8_array_t> make_serialized_data(const rcl_serialized_message_t& rcl_msg) {
    size_t buf_size = rcl_msg.buffer_length;
    auto* raw = static_cast<rcutils_uint8_array_t*>(std::malloc(sizeof(rcutils_uint8_array_t) + buf_size));
    raw->allocator = rcutils_get_default_allocator();
    raw->buffer = reinterpret_cast<uint8_t*>(raw + 1);
    raw->buffer_length = buf_size;
    raw->buffer_capacity = buf_size;
    std::memcpy(raw->buffer, rcl_msg.buffer, buf_size);
    return std::shared_ptr<rcutils_uint8_array_t>(raw, [](rcutils_uint8_array_t* m) { std::free(m); });
}

void SensorDataRecorder::removeRosbag2SqliteArtifactsAtUri(const std::string& uri)
{
    namespace fs = std::filesystem;
    auto ends_with = [](const std::string& s, const char* suf) -> bool {
        const size_t n = std::strlen(suf);
        return s.size() >= n && s.compare(s.size() - n, n, suf) == 0;
    };
    std::error_code ec;
    const fs::path root(uri);
    if (!fs::is_directory(root, ec) || ec) {
        return;
    }
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec) {
            break;
        }
        if (!entry.is_regular_file()) {
            continue;
        }
        const fs::path& p = entry.path();
        const std::string fname = p.filename().string();
        const std::string ext = p.extension().string();
        const bool is_rosbag2_sqlite =
            (fname == "metadata.yaml") || (ext == ".db3") || ends_with(fname, ".db3-wal") ||
            ends_with(fname, ".db3-shm");
        if (!is_rosbag2_sqlite) {
            continue;
        }
        ec.clear();
        fs::remove(p, ec);
        if (!ec) {
            RCLCPP_INFO(this->get_logger(),
                "Removed previous rosbag2 artifact before reopen: %s", p.string().c_str());
        }
    }
}

SensorDataRecorder::SensorDataRecorder(const rclcpp::NodeOptions& options)
    : rclcpp::Node("sensor_data_recorder", options)
{
    auto logger = rclcpp::get_logger("sensor_data_recorder");
    RCLCPP_INFO(logger, "Sensor Data Recorder starting...");
    
    // 加载配置
    loadConfig();
    // 启动时先加载本地缓存的绑定/隐私状态，避免等待云端下发期间全部被门控丢弃。
    load();
    
    // 创建录制目录
    createBagDirectory();
    
    // 不再订阅 cameraH264Topic，H264 文件直接在 h264_publisher 中落盘
    // camera_sub_ 已移除
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imuTopic, 10,
        std::bind(&SensorDataRecorder::imuCallback, this, std::placeholders::_1));
    
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gpsTopic, 10,
        std::bind(&SensorDataRecorder::gpsCallback, this, std::placeholders::_1));
    
    // 订阅CAN数据 - 需要从handle_can_adapter发布的话题订阅
    // 这里假设CAN数据会发布到一个话题，如果没有，需要先创建发布器
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/can/frames", rclcpp::QoS(rclcpp::KeepLast(100)).best_effort(),
        std::bind(&SensorDataRecorder::canCallback, this, std::placeholders::_1));
    
    // 订阅雷达数据
    lidar_right_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&SensorDataRecorder::lidarRightCallback, this, std::placeholders::_1));
    
    lidar_left_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&SensorDataRecorder::lidarLeftCallback, this, std::placeholders::_1));
    
    lidar_left_tilt_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftTiltLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&SensorDataRecorder::lidarLeftTiltCallback, this, std::placeholders::_1));
    
    lidar_right_tilt_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightTiltLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&SensorDataRecorder::lidarRightTiltCallback, this, std::placeholders::_1));
    
    lidar_back_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        backLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&SensorDataRecorder::lidarBackCallback, this, std::placeholders::_1));
    
    device_bind_status_sub_ = this->create_subscription<interface::msg::VehicleBindStatus>(
        cloudManagerDeviceBindPubTopic, 10,
        [this](const interface::msg::VehicleBindStatus::SharedPtr msg) { onDeviceBindStatus(msg); });
    
    // 注册参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&SensorDataRecorder::onSetParameters, this, std::placeholders::_1));
    
    // 创建定时器检查bag文件
    bag_check_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),  // 每5秒检查一次，确保及时检测到时长超限
        std::bind(&SensorDataRecorder::onBagCheckTimer, this));
    
    // 创建雷达数据定时器检查bag文件
    lidar_bag_check_timer_ = this->create_wall_timer(
        std::chrono::seconds(5),  // 每5秒检查一次，确保及时检测到时长超限
        std::bind(&SensorDataRecorder::onLidarBagCheckTimer, this));
    
    // 启动后台清理线程：每分钟扫描一次过期 bag 文件夹。原本使用 wall_timer 在 executor
    // 中执行 recursive_directory_iterator + file_size，单次可耗时数百毫秒~数秒，会直接
    // 阻塞 chassis 心跳定时器；现在迁移到独立线程，与 ROS executor 完全解耦。
    cleanup_thread_ = std::thread(&SensorDataRecorder::cleanupLoop, this);

    // 启动异步写盘工作线程：sensor 与 lidar 各一条，回调只做序列化和入队。
    sensor_writer_thread_ = std::thread(&SensorDataRecorder::sensorWriterLoop, this);
    lidar_writer_thread_ = std::thread(&SensorDataRecorder::lidarWriterLoop, this);
    
    // 创建服务用于控制录制
    start_record_service_ = this->create_service<std_srvs::srv::Empty>(
        "/sensor/recorder/start",
        std::bind(&SensorDataRecorder::startRecordServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    stop_record_service_ = this->create_service<std_srvs::srv::Empty>(
        "/sensor/recorder/stop",
        std::bind(&SensorDataRecorder::stopRecordServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    // 创建带路径的录制服务（用于与 H264 文件同步）
    start_record_with_path_service_ = this->create_service<interface::srv::StartRecordingWithPath>(
        "/sensor/recorder/start_with_path",
        std::bind(&SensorDataRecorder::startRecordWithPathServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    // 创建雷达数据录制服务
    start_lidar_record_service_ = this->create_service<std_srvs::srv::Empty>(
        "/sensor/recorder/lidar/start",
        std::bind(&SensorDataRecorder::startLidarRecordServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    stop_lidar_record_service_ = this->create_service<std_srvs::srv::Empty>(
        "/sensor/recorder/lidar/stop",
        std::bind(&SensorDataRecorder::stopLidarRecordServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    // 如果配置了自动开始录制，则启动录制
    if (auto_start_recording_) {
        startRecording();
    }
    
    // 如果配置了自动开始雷达数据录制，则启动录制
    if (auto_start_lidar_recording_) {
        startLidarRecording();
    }
    
    RCLCPP_INFO(logger, "Sensor Data Recorder initialized successfully!");
    RCLCPP_INFO(logger, "Record directory: %s", record_dir_.c_str());
    RCLCPP_INFO(logger, "Auto start recording: %s", auto_start_recording_ ? "true" : "false");
    RCLCPP_INFO(logger, "Auto start lidar recording: %s", auto_start_lidar_recording_ ? "true" : "false");
    RCLCPP_INFO(logger, "Sensor data - Max bag duration: %d minutes, Max bag size: %d MB", 
               max_bag_duration_minutes_, max_bag_size_mb_);
    RCLCPP_INFO(logger, "Lidar data - Max bag duration: %d minutes, Max bag size: %d MB", 
               max_lidar_bag_duration_minutes_, max_lidar_bag_size_mb_);
    RCLCPP_INFO(logger, "File retention: %d minutes (ring buffer)", retention_minutes_);
    RCLCPP_INFO(logger, "Lidar bag prefix: %s", lidar_bag_prefix_.c_str());
    RCLCPP_INFO(logger, "Camera verify mode: %s", verify_camera_enable_ ? "enabled" : "disabled");
}

SensorDataRecorder::~SensorDataRecorder()
{
    // 1) 先通知所有后台线程退出。
    writer_threads_stop_.store(true);
    cleanup_stop_.store(true);
    sensor_queue_cv_.notify_all();
    lidar_queue_cv_.notify_all();
    cleanup_cv_.notify_all();

    // 2) join 后台线程；这里必须先 join 写盘线程，确保不再触碰 bag_writer_，
    //    再 stopRecording 才安全。
    if (sensor_writer_thread_.joinable()) {
        sensor_writer_thread_.join();
    }
    if (lidar_writer_thread_.joinable()) {
        lidar_writer_thread_.join();
    }
    if (cleanup_thread_.joinable()) {
        cleanup_thread_.join();
    }

    stopRecording();
    stopLidarRecording();
}

void SensorDataRecorder::loadConfig()
{
    // 声明并获取参数
    this->declare_parameter("record.directory", std::string("/opt/freego/bags"));
    this->declare_parameter("record.bag_prefix", std::string("sensor_data"));
    this->declare_parameter("record.max_duration_minutes", 1);  // sensor数据时间分割（分钟）
    this->declare_parameter("record.max_size_mb", 1024);  // 1GB
    this->declare_parameter("record.auto_start", true);
    
    // 雷达数据录制配置
    this->declare_parameter("record.lidar.bag_prefix", std::string("lidar_data"));
    this->declare_parameter("record.lidar.max_duration_minutes", 2);  // lidar数据时间分割（分钟）
    this->declare_parameter("record.lidar.max_size_mb", 1024);  // 1GB
    this->declare_parameter("record.lidar.auto_start", true);
    
    // 环形缓冲区配置
    this->declare_parameter("record.retention_minutes", 30);  // 保留时长（分钟），默认30分钟
    
    // 验证模式配置
    this->declare_parameter("verify_camera.enable", false);
    this->declare_parameter("verify_camera.image_dir", std::string("./verify_images"));
    this->declare_parameter("verify_camera.frame_interval", 30);  // 每30帧保存一张图片
    
    record_dir_ = this->get_parameter("record.directory").as_string();
    bag_prefix_ = this->get_parameter("record.bag_prefix").as_string();
    max_bag_duration_minutes_ = this->get_parameter("record.max_duration_minutes").as_int();
    max_bag_size_mb_ = this->get_parameter("record.max_size_mb").as_int();
    auto_start_recording_ = this->get_parameter("record.auto_start").as_bool();
    
    lidar_bag_prefix_ = this->get_parameter("record.lidar.bag_prefix").as_string();
    max_lidar_bag_duration_minutes_ = this->get_parameter("record.lidar.max_duration_minutes").as_int();
    max_lidar_bag_size_mb_ = this->get_parameter("record.lidar.max_size_mb").as_int();
    auto_start_lidar_recording_ = this->get_parameter("record.lidar.auto_start").as_bool();
    
    retention_minutes_ = this->get_parameter("record.retention_minutes").as_int();
    
    verify_camera_enable_ = this->get_parameter("verify_camera.enable").as_bool();
    verify_image_dir_ = this->get_parameter("verify_camera.image_dir").as_string();
    verify_frame_interval_ = this->get_parameter("verify_camera.frame_interval").as_int();
    verify_frame_count_ = 0;
    
    // 初始化 SPS/PPS 缓存状态
    has_cached_sps_pps_ = false;
    cached_sps_.clear();
    cached_pps_.clear();
    
    // 如果启用了验证模式，创建图片保存目录
    if (verify_camera_enable_) {
        createVerifyImageDirectory();
        RCLCPP_INFO(this->get_logger(), "Camera验证模式已启用，图片保存目录: %s，帧间隔: %d", 
                   verify_image_dir_.c_str(), verify_frame_interval_);
    }
}

void SensorDataRecorder::createBagDirectory()
{
    std::filesystem::path record_path(record_dir_);
    if (!std::filesystem::exists(record_path)) {
        std::filesystem::create_directories(record_path);
        RCLCPP_INFO(this->get_logger(), "Created record directory: %s", record_dir_.c_str());
    }
}

std::string SensorDataRecorder::generateBagPath()
{
    // 生成毫秒级时间戳 + "sensor" 字符串的文件夹名（与 h264_publisher 保持一致）
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
    
    std::ostringstream oss;
    std::string sensor_folder = std::to_string(ms) + "sensor";
    oss << record_dir_ << "/" << sensor_folder << "/" << bag_prefix_;
    
    return oss.str();
}

std::string SensorDataRecorder::generateLidarBagPath()
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_info = std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << record_dir_ << "/" << lidar_bag_prefix_ << "_"
        << std::setfill('0') << std::setw(4) << (tm_info->tm_year + 1900)
        << std::setw(2) << (tm_info->tm_mon + 1)
        << std::setw(2) << tm_info->tm_mday << "_"
        << std::setw(2) << tm_info->tm_hour
        << std::setw(2) << tm_info->tm_min
        << std::setw(2) << tm_info->tm_sec;
    
    return oss.str();
}

void SensorDataRecorder::startRecording()
{
    startRecording(generateBagPath());
}

void SensorDataRecorder::startRecording(const std::string& folder_path)
{
    if (is_recording_.load()) {
        RCLCPP_WARN(this->get_logger(), "Recording is already started, stopping current recording first");
        sensor_writer_restarting_.store(true);
        stopRecording();
    }
    
    try {
        std::lock_guard<std::mutex> lock(writer_mutex_);
        // 使用指定的文件夹路径，直接在该文件夹中创建 bag 文件
        // 文件夹路径格式: /path/to/storage/sensor_20260114_104057
        // bag 文件路径: /path/to/storage/sensor_20260114_104057/sensor_data
        // 与 h264 文件共用同一个文件夹，不重新创建文件夹
        current_bag_path_ = folder_path;
        current_bag_start_time_ = std::chrono::system_clock::now();
        current_bag_size_bytes_ = 0;
        
        // 不创建文件夹，因为 h264_publisher 已经创建了该文件夹
        // 只检查文件夹是否存在，如果不存在则报错
        if (!std::filesystem::exists(record_dir_)) {
            RCLCPP_ERROR(this->get_logger(), "Folder does not exist: %s (should be created by h264_publisher)", record_dir_.c_str());
            sensor_writer_restarting_.store(false);
            return;
        }

        // 与 H264 共用会话目录时，重复 start_with_path/open 会因 sqlite 目录已存在而失败；
        // 仅移除 rosbag2 产物，保留 .h264 等。
        removeRosbag2SqliteArtifactsAtUri(current_bag_path_);
        
        // 配置存储选项
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = current_bag_path_;
        storage_options.storage_id = "sqlite3";  // 使用SQLite3格式，支持ros2 bag play
        
        // 配置转换选项
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        // 创建rosbag2 writer
        bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
        bag_writer_->open(storage_options, converter_options);
        
        // 创建所有话题的元数据
        rosbag2_storage::TopicMetadata topic_metadata;
        
        // 不再录制 Camera 话题，H264 文件直接在 h264_publisher 中落盘
        
        // IMU话题
        topic_metadata.name = imuTopic;
        topic_metadata.type = "sensor_msgs/msg/Imu";
        topic_metadata.serialization_format = "cdr";
        bag_writer_->create_topic(topic_metadata);
        
        // GPS话题
        topic_metadata.name = gpsTopic;
        topic_metadata.type = "sensor_msgs/msg/NavSatFix";
        topic_metadata.serialization_format = "cdr";
        bag_writer_->create_topic(topic_metadata);
        
        // CAN话题
        topic_metadata.name = "/can/frames";
        topic_metadata.type = "can_msgs/msg/Frame";
        topic_metadata.serialization_format = "cdr";
        bag_writer_->create_topic(topic_metadata);
        
        is_recording_.store(true);
        
        RCLCPP_INFO(this->get_logger(), "Started recording to: %s (in folder: %s)", 
                   current_bag_path_.c_str(), folder_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Recording topics: %s, %s, /can/frames (H264 files saved separately)",
                   imuTopic, gpsTopic);
        sensor_writer_restarting_.store(false);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
        if (bag_writer_) {
            bag_writer_.reset();
        }
        sensor_writer_restarting_.store(false);
    }
}

void SensorDataRecorder::stopRecording()
{
    std::lock_guard<std::mutex> lock(writer_mutex_);
    
    if (!is_recording_.load()) {
        return;
    }
    
    try {
        if (bag_writer_) {
            bag_writer_->close();
            bag_writer_.reset();
        }
        
        is_recording_.store(false);
        
        RCLCPP_INFO(this->get_logger(), "Stopped recording. Bag file saved to: %s", 
                   current_bag_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "You can play it with: ros2 bag play %s", 
                   current_bag_path_.c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop recording: %s", e.what());
    }
}

void SensorDataRecorder::writeMessage(const std::string& topic_name, 
                                     const std::string& topic_type,
                                     const rclcpp::Time& timestamp,
                                     const void* msg_ptr)
{
    // 回调路径只做序列化与入队，写盘交给 sensorWriterLoop。
    if (!is_recording_.load()) {
        return;
    }

    try {
        rclcpp::SerializedMessage serialized_msg;
        if (topic_type == "sensor_msgs/msg/Imu") {
            const auto* msg = static_cast<const sensor_msgs::msg::Imu*>(msg_ptr);
            imu_serialization_.serialize_message(msg, &serialized_msg);
        } else if (topic_type == "sensor_msgs/msg/NavSatFix") {
            const auto* msg = static_cast<const sensor_msgs::msg::NavSatFix*>(msg_ptr);
            gps_serialization_.serialize_message(msg, &serialized_msg);
        } else if (topic_type == "can_msgs/msg/Frame") {
            const auto* msg = static_cast<const can_msgs::msg::Frame*>(msg_ptr);
            can_serialization_.serialize_message(msg, &serialized_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown topic type: %s", topic_type.c_str());
            return;
        }

        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data = make_serialized_data(serialized_msg.get_rcl_serialized_message());
        bag_message->time_stamp = timestamp.nanoseconds();
        bag_message->topic_name = topic_name;

        enqueueSensorMessage({std::move(bag_message)});

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to serialize sensor message: %s", e.what());
    }
}

// cameraCallback 已移除，H264 文件直接在 h264_publisher 中落盘

void SensorDataRecorder::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Location") != std::string::npos) {
        writeMessage(imuTopic, "sensor_msgs/msg/Imu", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传IMU数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
}

void SensorDataRecorder::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Location") != std::string::npos) {
        writeMessage(gpsTopic, "sensor_msgs/msg/NavSatFix", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传GPS数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
    
}

void SensorDataRecorder::canCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Assist") != std::string::npos) {
        writeMessage("/can/frames", "can_msgs/msg/Frame", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传辅助驾驶数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
    
}

void SensorDataRecorder::lidarRightCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Assist") != std::string::npos) {
        writeLidarMessage("/scan/right", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传辅助驾驶数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
}

void SensorDataRecorder::lidarLeftCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Assist") != std::string::npos) {
        writeLidarMessage("/scan/left", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传辅助驾驶数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
}

void SensorDataRecorder::lidarLeftTiltCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Assist") != std::string::npos) {
        writeLidarMessage("/scan/left_tilt", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传辅助驾驶数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
}

void SensorDataRecorder::lidarRightTiltCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Assist") != std::string::npos) {
        writeLidarMessage("/scan/right_tilt", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传辅助驾驶数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
}

void SensorDataRecorder::lidarBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (app_bind_page_state_ == 1 && privacy_policy_.find("Assist") != std::string::npos) {
        writeLidarMessage("/scan/back", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
    } else {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "用户未绑定或者选择不上传辅助驾驶数据:app_bind_page_state：%d, privacy_policy：%s",
            app_bind_page_state_, privacy_policy_.c_str());
    }
}

void SensorDataRecorder::onBagCheckTimer()
{
    if (!is_recording_.load()) {
        return;
    }
    
    // 检查是否需要创建新的bag文件
    bool need_new_bag = false;
    
    // 检查时长 - 使用秒级精度，避免分钟级向下取整导致的误差
    auto now = std::chrono::system_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::seconds>(
        now - current_bag_start_time_).count();
    int64_t max_duration_seconds = static_cast<int64_t>(max_bag_duration_minutes_) * 60;
    
    if (duration_seconds >= max_duration_seconds) {
        need_new_bag = true;
        RCLCPP_INFO(this->get_logger(), "Bag file duration (%ld seconds, %.2f minutes) exceeds limit (%d minutes), creating new bag",
                   duration_seconds, duration_seconds / 60.0, max_bag_duration_minutes_);
    }
    
    // 检查大小
    int64_t max_size_bytes = static_cast<int64_t>(max_bag_size_mb_) * 1024 * 1024;
    if (current_bag_size_bytes_ >= max_size_bytes) {
        need_new_bag = true;
        RCLCPP_INFO(this->get_logger(), "Bag file size (%ld bytes) exceeds limit (%d MB), creating new bag",
                   current_bag_size_bytes_, max_bag_size_mb_);
    }
    
    if (need_new_bag) {
        // 停止当前录制并开始新的
        sensor_writer_restarting_.store(true);
        stopRecording();
        startRecording();
    }
}

rcl_interfaces::msg::SetParametersResult SensorDataRecorder::onSetParameters(
    const std::vector<rclcpp::Parameter>& parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto& param : parameters) {
        if (param.get_name() == "record.directory") {
            record_dir_ = param.as_string();
            createBagDirectory();
            RCLCPP_INFO(this->get_logger(), "Record directory updated to: %s", record_dir_.c_str());
        } else if (param.get_name() == "record.bag_prefix") {
            bag_prefix_ = param.as_string();
            RCLCPP_INFO(this->get_logger(), "Bag prefix updated to: %s", bag_prefix_.c_str());
        } else if (param.get_name() == "record.max_duration_minutes") {
            max_bag_duration_minutes_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Max bag duration updated to: %d minutes", 
                       max_bag_duration_minutes_);
        } else if (param.get_name() == "record.max_size_mb") {
            max_bag_size_mb_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Max bag size updated to: %d MB", max_bag_size_mb_);
        } else if (param.get_name() == "record.auto_start") {
            auto_start_recording_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Auto start recording updated to: %s", 
                       auto_start_recording_ ? "true" : "false");
        } else if (param.get_name() == "record.lidar.bag_prefix") {
            lidar_bag_prefix_ = param.as_string();
            RCLCPP_INFO(this->get_logger(), "Lidar bag prefix updated to: %s", lidar_bag_prefix_.c_str());
        } else if (param.get_name() == "record.lidar.max_duration_minutes") {
            max_lidar_bag_duration_minutes_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Max lidar bag duration updated to: %d minutes", 
                       max_lidar_bag_duration_minutes_);
        } else if (param.get_name() == "record.lidar.max_size_mb") {
            max_lidar_bag_size_mb_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Max lidar bag size updated to: %d MB", max_lidar_bag_size_mb_);
        } else if (param.get_name() == "record.lidar.auto_start") {
            auto_start_lidar_recording_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Auto start lidar recording updated to: %s", 
                       auto_start_lidar_recording_ ? "true" : "false");
        } else if (param.get_name() == "record.retention_minutes") {
            retention_minutes_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Retention minutes updated to: %d minutes", 
                       retention_minutes_);
        } else if (param.get_name() == "verify_camera.enable") {
            verify_camera_enable_ = param.as_bool();
            if (verify_camera_enable_) {
                createVerifyImageDirectory();
            }
            RCLCPP_INFO(this->get_logger(), "Camera verify mode updated to: %s", 
                       verify_camera_enable_ ? "enabled" : "disabled");
        } else if (param.get_name() == "verify_camera.image_dir") {
            verify_image_dir_ = param.as_string();
            createVerifyImageDirectory();
            RCLCPP_INFO(this->get_logger(), "Verify image directory updated to: %s", verify_image_dir_.c_str());
        } else if (param.get_name() == "verify_camera.frame_interval") {
            verify_frame_interval_ = param.as_int();
            RCLCPP_INFO(this->get_logger(), "Verify frame interval updated to: %d", verify_frame_interval_);
        }
    }
    
    return result;
}

void SensorDataRecorder::startRecordServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request_header;
    (void)request;
    (void)response;
    
    startRecording();
}

void SensorDataRecorder::stopRecordServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request_header;
    (void)request;
    (void)response;
    
    stopRecording();
}

void SensorDataRecorder::startRecordWithPathServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<interface::srv::StartRecordingWithPath::Request> request,
    std::shared_ptr<interface::srv::StartRecordingWithPath::Response> response)
{
    (void)request_header;
    
    try {
        if (request->folder_path.empty()) {
            response->success = false;
            response->message = "Folder path is empty";
            RCLCPP_ERROR(this->get_logger(), "Failed to start recording: folder path is empty");
            return;
        }
        
        startRecording(request->folder_path);
        response->success = true;
        response->message = "Recording started successfully in folder: " + request->folder_path;
        RCLCPP_INFO(this->get_logger(), "Recording started via service in folder: %s", 
                   request->folder_path.c_str());
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Failed to start recording: ") + e.what();
        RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
    }
}

void SensorDataRecorder::startLidarRecording()
{
    std::lock_guard<std::mutex> lock(lidar_writer_mutex_);
    
    if (is_lidar_recording_.load()) {
        RCLCPP_WARN(this->get_logger(), "Lidar recording is already started");
        return;
    }
    
    try {
        current_lidar_bag_path_ = generateLidarBagPath();
        current_lidar_bag_start_time_ = std::chrono::system_clock::now();
        current_lidar_bag_size_bytes_ = 0;
        
        // 配置存储选项
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = current_lidar_bag_path_;
        storage_options.storage_id = "sqlite3";  // 使用SQLite3格式，支持ros2 bag play
        
        // 配置转换选项
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        // 创建rosbag2 writer
        lidar_bag_writer_ = std::make_unique<rosbag2_cpp::Writer>();
        lidar_bag_writer_->open(storage_options, converter_options);
        
        // 创建所有雷达话题的元数据
        rosbag2_storage::TopicMetadata topic_metadata;
        
        // 雷达话题
        topic_metadata.name = "/scan/right";
        topic_metadata.type = "sensor_msgs/msg/LaserScan";
        topic_metadata.serialization_format = "cdr";
        lidar_bag_writer_->create_topic(topic_metadata);
        
        topic_metadata.name = "/scan/left";
        topic_metadata.type = "sensor_msgs/msg/LaserScan";
        topic_metadata.serialization_format = "cdr";
        lidar_bag_writer_->create_topic(topic_metadata);
        
        topic_metadata.name = "/scan/left_tilt";
        topic_metadata.type = "sensor_msgs/msg/LaserScan";
        topic_metadata.serialization_format = "cdr";
        lidar_bag_writer_->create_topic(topic_metadata);
        
        topic_metadata.name = "/scan/right_tilt";
        topic_metadata.type = "sensor_msgs/msg/LaserScan";
        topic_metadata.serialization_format = "cdr";
        lidar_bag_writer_->create_topic(topic_metadata);
        
        topic_metadata.name = "/scan/back";
        topic_metadata.type = "sensor_msgs/msg/LaserScan";
        topic_metadata.serialization_format = "cdr";
        lidar_bag_writer_->create_topic(topic_metadata);
        
        is_lidar_recording_.store(true);
        
        RCLCPP_INFO(this->get_logger(), "Started lidar recording to: %s", current_lidar_bag_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Recording lidar topics: /scan/right, /scan/left, /scan/left_tilt, /scan/right_tilt, /scan/back");
        lidar_writer_restarting_.store(false);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start lidar recording: %s", e.what());
        if (lidar_bag_writer_) {
            lidar_bag_writer_.reset();
        }
        lidar_writer_restarting_.store(false);
    }
}

void SensorDataRecorder::stopLidarRecording()
{
    std::lock_guard<std::mutex> lock(lidar_writer_mutex_);
    
    if (!is_lidar_recording_.load()) {
        return;
    }
    
    try {
        if (lidar_bag_writer_) {
            lidar_bag_writer_->close();
            lidar_bag_writer_.reset();
        }
        
        is_lidar_recording_.store(false);
        
        RCLCPP_INFO(this->get_logger(), "Stopped lidar recording. Bag file saved to: %s", 
                   current_lidar_bag_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "You can play it with: ros2 bag play %s", 
                   current_lidar_bag_path_.c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to stop lidar recording: %s", e.what());
    }
}

void SensorDataRecorder::writeLidarMessage(const std::string& topic_name, 
                                           const std::string& topic_type,
                                           const rclcpp::Time& timestamp,
                                           const void* msg_ptr)
{
    // 回调路径只做序列化与入队，写盘交给 lidarWriterLoop。
    if (!is_lidar_recording_.load()) {
        return;
    }

    try {
        rclcpp::SerializedMessage serialized_msg;
        if (topic_type == "sensor_msgs/msg/LaserScan") {
            const auto* msg = static_cast<const sensor_msgs::msg::LaserScan*>(msg_ptr);
            lidar_serialization_.serialize_message(msg, &serialized_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown lidar topic type: %s", topic_type.c_str());
            return;
        }

        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->serialized_data = make_serialized_data(serialized_msg.get_rcl_serialized_message());
        bag_message->time_stamp = timestamp.nanoseconds();
        bag_message->topic_name = topic_name;

        enqueueLidarMessage({std::move(bag_message)});

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to serialize lidar message: %s", e.what());
    }
}

void SensorDataRecorder::onLidarBagCheckTimer()
{
    if (!is_lidar_recording_.load()) {
        return;
    }
    
    // 检查是否需要创建新的bag文件
    bool need_new_bag = false;
    
    // 检查时长 - 使用秒级精度，避免分钟级向下取整导致的误差
    // 使用 lidar 独立的时间配置
    auto now = std::chrono::system_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::seconds>(
        now - current_lidar_bag_start_time_).count();
    int64_t max_duration_seconds = static_cast<int64_t>(max_lidar_bag_duration_minutes_) * 60;
    
    if (duration_seconds >= max_duration_seconds) {
        need_new_bag = true;
        RCLCPP_INFO(this->get_logger(), "Lidar bag file duration (%ld seconds, %.2f minutes) exceeds limit (%d minutes), creating new bag",
                   duration_seconds, duration_seconds / 60.0, max_lidar_bag_duration_minutes_);
    }
    
    // 检查大小 - 使用 lidar 独立的大小配置
    int64_t max_size_bytes = static_cast<int64_t>(max_lidar_bag_size_mb_) * 1024 * 1024;
    if (current_lidar_bag_size_bytes_ >= max_size_bytes) {
        need_new_bag = true;
        RCLCPP_INFO(this->get_logger(), "Lidar bag file size (%ld bytes) exceeds limit (%d MB), creating new bag",
                   current_lidar_bag_size_bytes_, max_lidar_bag_size_mb_);
    }
    
    if (need_new_bag) {
        // 停止当前录制并开始新的
        lidar_writer_restarting_.store(true);
        stopLidarRecording();
        startLidarRecording();
    }
}

// onCleanupTimer 已删除：清理工作迁移到 cleanupLoop（独立线程），不再阻塞 ROS executor。

void SensorDataRecorder::cleanupLoop()
{
    auto logger = this->get_logger();
    RCLCPP_INFO(logger, "cleanupLoop started");
    while (!cleanup_stop_.load()) {
        {
            std::unique_lock<std::mutex> lk(cleanup_cv_mutex_);
            // 与 ROS timer 等价的 1 分钟周期；析构时通过 cleanup_cv_ + cleanup_stop_ 立刻唤醒退出。
            cleanup_cv_.wait_for(lk, std::chrono::minutes(1),
                                 [this] { return cleanup_stop_.load(); });
        }
        if (cleanup_stop_.load()) {
            break;
        }
        try {
            cleanupOldFiles();
        } catch (const std::exception& e) {
            RCLCPP_ERROR(logger, "cleanupLoop exception: %s", e.what());
        }
    }
    RCLCPP_INFO(logger, "cleanupLoop stopped");
}

void SensorDataRecorder::sensorWriterLoop()
{
    auto logger = this->get_logger();
    RCLCPP_INFO(logger, "sensorWriterLoop started");
    std::deque<PendingBagMessage> batch;
    while (true) {
        {
            std::unique_lock<std::mutex> lk(sensor_queue_mutex_);
            sensor_queue_cv_.wait_for(lk, std::chrono::milliseconds(100),
                [this] { return writer_threads_stop_.load() || !sensor_queue_.empty(); });
            std::swap(batch, sensor_queue_);
        }

        if (!batch.empty()) {
            bool should_requeue = false;
            std::lock_guard<std::mutex> lock(writer_mutex_);
            if (is_recording_.load() && bag_writer_) {
                for (auto& m : batch) {
                    try {
                        bag_writer_->write(m.bag_msg);
                        current_bag_size_bytes_ += m.bag_msg->serialized_data->buffer_length;
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(logger, "sensor bag write failed: %s", e.what());
                    }
                }
            } else if (sensor_writer_restarting_.load() && !writer_threads_stop_.load()) {
                should_requeue = true;
            }
            if (should_requeue) {
                std::lock_guard<std::mutex> qlk(sensor_queue_mutex_);
                for (auto it = batch.rbegin(); it != batch.rend(); ++it) {
                    if (sensor_queue_.size() >= kMaxPendingQueueSize) {
                        sensor_queue_.pop_front();
                        sensor_dropped_msgs_.fetch_add(1, std::memory_order_relaxed);
                    }
                    sensor_queue_.push_front(std::move(*it));
                }
            }
            batch.clear();
        }

        if (writer_threads_stop_.load()) {
            // 停机时排空残余消息：仅当还在录制时尝试落盘。
            std::unique_lock<std::mutex> lk(sensor_queue_mutex_);
            if (sensor_queue_.empty()) {
                break;
            }
            std::swap(batch, sensor_queue_);
            lk.unlock();
            std::lock_guard<std::mutex> lock(writer_mutex_);
            if (is_recording_.load() && bag_writer_) {
                for (auto& m : batch) {
                    try {
                        bag_writer_->write(m.bag_msg);
                    } catch (...) {}
                }
            }
            batch.clear();
            break;
        }
    }
    auto dropped = sensor_dropped_msgs_.load();
    if (dropped > 0) {
        RCLCPP_WARN(logger, "sensorWriterLoop stopped, total dropped=%lu", static_cast<unsigned long>(dropped));
    } else {
        RCLCPP_INFO(logger, "sensorWriterLoop stopped");
    }
}

void SensorDataRecorder::lidarWriterLoop()
{
    auto logger = this->get_logger();
    RCLCPP_INFO(logger, "lidarWriterLoop started");
    std::deque<PendingBagMessage> batch;
    while (true) {
        {
            std::unique_lock<std::mutex> lk(lidar_queue_mutex_);
            lidar_queue_cv_.wait_for(lk, std::chrono::milliseconds(100),
                [this] { return writer_threads_stop_.load() || !lidar_queue_.empty(); });
            std::swap(batch, lidar_queue_);
        }

        if (!batch.empty()) {
            bool should_requeue = false;
            std::lock_guard<std::mutex> lock(lidar_writer_mutex_);
            if (is_lidar_recording_.load() && lidar_bag_writer_) {
                for (auto& m : batch) {
                    try {
                        lidar_bag_writer_->write(m.bag_msg);
                        current_lidar_bag_size_bytes_ += m.bag_msg->serialized_data->buffer_length;
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(logger, "lidar bag write failed: %s", e.what());
                    }
                }
            } else if (lidar_writer_restarting_.load() && !writer_threads_stop_.load()) {
                should_requeue = true;
            }
            if (should_requeue) {
                std::lock_guard<std::mutex> qlk(lidar_queue_mutex_);
                for (auto it = batch.rbegin(); it != batch.rend(); ++it) {
                    if (lidar_queue_.size() >= kMaxPendingQueueSize) {
                        lidar_queue_.pop_front();
                        lidar_dropped_msgs_.fetch_add(1, std::memory_order_relaxed);
                    }
                    lidar_queue_.push_front(std::move(*it));
                }
            }
            batch.clear();
        }

        if (writer_threads_stop_.load()) {
            std::unique_lock<std::mutex> lk(lidar_queue_mutex_);
            if (lidar_queue_.empty()) {
                break;
            }
            std::swap(batch, lidar_queue_);
            lk.unlock();
            std::lock_guard<std::mutex> lock(lidar_writer_mutex_);
            if (is_lidar_recording_.load() && lidar_bag_writer_) {
                for (auto& m : batch) {
                    try {
                        lidar_bag_writer_->write(m.bag_msg);
                    } catch (...) {}
                }
            }
            batch.clear();
            break;
        }
    }
    auto dropped = lidar_dropped_msgs_.load();
    if (dropped > 0) {
        RCLCPP_WARN(logger, "lidarWriterLoop stopped, total dropped=%lu", static_cast<unsigned long>(dropped));
    } else {
        RCLCPP_INFO(logger, "lidarWriterLoop stopped");
    }
}

void SensorDataRecorder::enqueueSensorMessage(PendingBagMessage&& msg)
{
    {
        std::lock_guard<std::mutex> lk(sensor_queue_mutex_);
        if (sensor_queue_.size() >= kMaxPendingQueueSize) {
            // 队列爆掉时丢弃最旧消息以保护内存；正常机器不应触发，仅做兜底。
            sensor_queue_.pop_front();
            sensor_dropped_msgs_.fetch_add(1, std::memory_order_relaxed);
        }
        sensor_queue_.emplace_back(std::move(msg));
    }
    sensor_queue_cv_.notify_one();
}

void SensorDataRecorder::enqueueLidarMessage(PendingBagMessage&& msg)
{
    {
        std::lock_guard<std::mutex> lk(lidar_queue_mutex_);
        if (lidar_queue_.size() >= kMaxPendingQueueSize) {
            lidar_queue_.pop_front();
            lidar_dropped_msgs_.fetch_add(1, std::memory_order_relaxed);
        }
        lidar_queue_.emplace_back(std::move(msg));
    }
    lidar_queue_cv_.notify_one();
}

void SensorDataRecorder::cleanupOldFiles()
{
    if (retention_minutes_ <= 0) {
        // 如果保留时长为0或负数，不进行清理
        return;
    }
    
    try {
        namespace fs = std::filesystem;
        auto now = std::chrono::system_clock::now();
        auto cutoff_time = now - std::chrono::minutes(retention_minutes_);
        
        if (!fs::exists(record_dir_)) {
            return;
        }
        
        int deleted_count = 0;
        int64_t freed_space_bytes = 0;
        
        // 遍历录制目录下的所有文件夹
        for (const auto& entry : fs::directory_iterator(record_dir_)) {
            if (!entry.is_directory()) {
                continue;
            }
            
            std::error_code ec;
            auto last_write_time = fs::last_write_time(entry.path(), ec);
            if (ec) {
                continue;
            }
            
            // 将文件系统时间转换为 system_clock 时间
            auto sctp = std::chrono::time_point_cast<std::chrono::system_clock::duration>(
                last_write_time - decltype(last_write_time)::clock::now() + 
                std::chrono::system_clock::now());
            
            // 如果文件夹的修改时间早于截止时间，删除它
            if (sctp < cutoff_time) {
                // 计算文件夹大小
                int64_t folder_size = 0;
                try {
                    for (const auto& sub_entry : fs::recursive_directory_iterator(entry.path())) {
                        if (sub_entry.is_regular_file()) {
                            folder_size += fs::file_size(sub_entry.path());
                        }
                    }
                } catch (...) {
                    // 忽略计算大小时的错误
                }
                
                // 删除文件夹
                try {
                    fs::remove_all(entry.path(), ec);
                    if (!ec) {
                        deleted_count++;
                        freed_space_bytes += folder_size;
                        RCLCPP_DEBUG(this->get_logger(), 
                                   "Deleted old folder: %s (size: %.2f MB)", 
                                   entry.path().c_str(), folder_size / (1024.0 * 1024.0));
                    }
                } catch (const std::exception& e) {
                    RCLCPP_WARN(this->get_logger(), 
                               "Failed to delete folder %s: %s", 
                               entry.path().c_str(), e.what());
                }
            }
        }
        
        if (deleted_count > 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "Cleaned up %d old folders, freed %.2f MB (retention: %d minutes)",
                       deleted_count, 
                       freed_space_bytes / (1024.0 * 1024.0),
                       retention_minutes_);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error during cleanup: %s", e.what());
    }
}

void SensorDataRecorder::startLidarRecordServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request_header;
    (void)request;
    (void)response;
    
    startLidarRecording();
}

void SensorDataRecorder::stopLidarRecordServiceCallback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
    (void)request_header;
    (void)request;
    (void)response;
    
    stopLidarRecording();
}

void SensorDataRecorder::createVerifyImageDirectory()
{
    std::filesystem::path image_path(verify_image_dir_);
    if (!std::filesystem::exists(image_path)) {
        std::filesystem::create_directories(image_path);
        RCLCPP_INFO(this->get_logger(), "创建验证图片目录: %s", verify_image_dir_.c_str());
    }
}

std::string SensorDataRecorder::generateImagePath(const rclcpp::Time& timestamp)
{
    auto time_t = std::chrono::system_clock::to_time_t(
        std::chrono::time_point<std::chrono::system_clock>(
            std::chrono::nanoseconds(timestamp.nanoseconds())));
    std::tm* tm_info = std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << verify_image_dir_ << "/camera_"
        << std::setfill('0') << std::setw(4) << (tm_info->tm_year + 1900)
        << std::setw(2) << (tm_info->tm_mon + 1)
        << std::setw(2) << tm_info->tm_mday << "_"
        << std::setw(2) << tm_info->tm_hour
        << std::setw(2) << tm_info->tm_min
        << std::setw(2) << tm_info->tm_sec << "_"
        << std::setw(9) << (timestamp.nanoseconds() % 1000000000)
        << ".jpg";
    
    return oss.str();
}

bool SensorDataRecorder::hasValidH264StartCode(const std::vector<uint8_t>& h264_data)
{
    if (h264_data.size() < 4) {
        return false;
    }
    
    // 检查是否包含H264起始码（0x00000001或0x000001）
    for (size_t i = 0; i < h264_data.size() - 3; i++) {
        // 检查4字节起始码 0x00000001
        if (h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && 
            h264_data[i+2] == 0x00 && h264_data[i+3] == 0x01) {
            return true;
        }
        // 检查3字节起始码 0x000001
        if (i < h264_data.size() - 2 && 
            h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && h264_data[i+2] == 0x01) {
            return true;
        }
    }
    
    return false;
}

bool SensorDataRecorder::hasIDRFrame(const std::vector<uint8_t>& h264_data)
{
    if (h264_data.size() < 5) {
        return false;
    }
    
    // 查找IDR帧（NAL类型5）
    // IDR帧的NAL单元类型为5，位于起始码之后
    for (size_t i = 0; i < h264_data.size() - 4; i++) {
        // 检查4字节起始码 0x00000001
        if (h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && 
            h264_data[i+2] == 0x00 && h264_data[i+3] == 0x01) {
            if (i + 4 < h264_data.size()) {
                unsigned char nal_type = h264_data[i + 4] & 0x1F; // NAL类型在低5位
                if (nal_type == 5) {  // IDR帧
                    return true;
                }
            }
        }
        // 检查3字节起始码 0x000001
        if (i < h264_data.size() - 3 && 
            h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && h264_data[i+2] == 0x01) {
            if (i + 3 < h264_data.size()) {
                unsigned char nal_type = h264_data[i + 3] & 0x1F;
                if (nal_type == 5) {  // IDR帧
                    return true;
                }
            }
        }
    }
    
    return false;
}

void SensorDataRecorder::decodeAndSaveImage(const std::vector<uint8_t>& h264_data, const rclcpp::Time& timestamp)
{
    if (h264_data.empty()) {
        RCLCPP_WARN(this->get_logger(), "H264数据为空，跳过解码");
        return;
    }
    
    // 检查数据大小，避免处理过大的数据
    const size_t MAX_H264_SIZE = 10 * 1024 * 1024;  // 10MB限制
    if (h264_data.size() > MAX_H264_SIZE) {
        RCLCPP_WARN(this->get_logger(), "H264数据过大 (%zu 字节)，跳过解码", h264_data.size());
        return;
    }
    
    // 再次验证数据有效性
    if (!hasValidH264StartCode(h264_data)) {
        RCLCPP_WARN(this->get_logger(), "H264数据不包含有效的起始码，跳过解码");
        return;
    }
    
    // 生成图片保存路径
    std::string image_path = generateImagePath(timestamp);
    
    // 创建临时H264文件
    std::string temp_h264_path = image_path + ".tmp.h264";
    std::ofstream temp_file(temp_h264_path, std::ios::binary);
    if (!temp_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "无法创建临时H264文件: %s", temp_h264_path.c_str());
        return;
    }
    
    // 写入H264数据
    temp_file.write(reinterpret_cast<const char*>(h264_data.data()), h264_data.size());
    temp_file.close();
    
    // 使用FFmpeg将H264解码为JPEG图片
    // 改进的命令参数：
    // -y: 覆盖输出文件
    // -f h264: 明确指定输入格式为H264
    // -i: 输入文件
    // -vframes 1: 只解码第一帧（等同于-frames:v 1）
    // -q:v 2: 高质量JPEG (2-31, 2为最高质量)
    // -pix_fmt yuvj420p: 确保颜色空间正确
    // 将错误输出重定向到临时文件以便调试
    std::string error_log = image_path + ".ffmpeg_error.log";
    std::ostringstream cmd;
    cmd << "ffmpeg -y -f h264 -i " << temp_h264_path 
        << " -vframes 1 -q:v 2 -pix_fmt yuvj420p " << image_path 
        << " 2>" << error_log;
    
    int ret = system(cmd.str().c_str());
    
    // 如果失败，读取错误日志
    if (ret != 0) {
        std::ifstream error_file(error_log);
        if (error_file.is_open()) {
            std::string error_content((std::istreambuf_iterator<char>(error_file)),
                                     std::istreambuf_iterator<char>());
            error_file.close();
            if (!error_content.empty()) {
                RCLCPP_WARN(this->get_logger(), "FFmpeg错误信息: %s", error_content.c_str());
            }
        }
        std::remove(error_log.c_str());
    }
    
    // 删除临时文件
    if (std::remove(temp_h264_path.c_str()) != 0) {
        RCLCPP_DEBUG(this->get_logger(), "删除临时文件失败: %s", temp_h264_path.c_str());
    }
    
    if (ret == 0) {
        // 检查输出文件是否存在且大小合理
        std::ifstream check_file(image_path, std::ios::binary | std::ios::ate);
        if (check_file.is_open()) {
            std::streamsize file_size = check_file.tellg();
            check_file.close();
            if (file_size > 0) {
                RCLCPP_INFO(this->get_logger(), "成功解码并保存图片: %s (大小: %ld 字节)", 
                           image_path.c_str(), file_size);
            } else {
                RCLCPP_WARN(this->get_logger(), "解码后的图片文件为空: %s", image_path.c_str());
                std::remove(image_path.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "解码后的图片文件不存在: %s", image_path.c_str());
        }
    } else {
        RCLCPP_WARN(this->get_logger(), "H264解码失败，返回码: %d，数据大小: %zu 字节，图片路径: %s", 
                   ret, h264_data.size(), image_path.c_str());
        // 如果解码失败，尝试删除可能创建的空文件
        std::remove(image_path.c_str());
    }
}

bool SensorDataRecorder::containsSPSPPS(const std::vector<uint8_t>& h264_data)
{
    // 检查是否包含 SPS (NAL 类型 7) 或 PPS (NAL 类型 8)
    for (size_t i = 0; i + 4 < h264_data.size(); ++i) {
        // 检查4字节起始码 0x00000001
        if (h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && 
            h264_data[i+2] == 0x00 && h264_data[i+3] == 0x01) {
            if (i + 4 < h264_data.size()) {
                uint8_t nal = h264_data[i + 4] & 0x1F;
                if (nal == 7 || nal == 8) {  // SPS (7) or PPS (8)
                    return true;
                }
            }
        }
        // 检查3字节起始码 0x000001
        if (i + 3 < h264_data.size() && 
            h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && h264_data[i+2] == 0x01) {
            uint8_t nal = h264_data[i+3] & 0x1F;
            if (nal == 7 || nal == 8) {  // SPS (7) or PPS (8)
                return true;
            }
        }
    }
    return false;
}

void SensorDataRecorder::cacheSPSPPS(const std::vector<uint8_t>& h264_data)
{
    std::lock_guard<std::mutex> lock(sps_pps_mutex_);
    
    // 查找并缓存 SPS 和 PPS 参数集
    for (size_t i = 0; i + 4 < h264_data.size(); ++i) {
        size_t start_code_pos = i;
        size_t nalu_start = 0;
        bool found_start_code = false;
        
        // 检查4字节起始码 0x00000001
        if (h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && 
            h264_data[i+2] == 0x00 && h264_data[i+3] == 0x01) {
            nalu_start = i + 4;
            found_start_code = true;
        }
        // 检查3字节起始码 0x000001
        else if (i + 3 < h264_data.size() && 
                 h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && h264_data[i+2] == 0x01) {
            nalu_start = i + 3;
            found_start_code = true;
        }
        
        if (found_start_code && nalu_start < h264_data.size()) {
            uint8_t nal = h264_data[nalu_start] & 0x1F;
            
            if (nal == 7) {  // SPS
                // 找到下一个起始码，确定 SPS 的结束位置
                size_t next_start_code = h264_data.size();
                for (size_t j = nalu_start + 1; j + 3 < h264_data.size(); ++j) {
                    if ((h264_data[j] == 0x00 && h264_data[j+1] == 0x00 && 
                         h264_data[j+2] == 0x00 && h264_data[j+3] == 0x01) ||
                        (j + 2 < h264_data.size() && 
                         h264_data[j] == 0x00 && h264_data[j+1] == 0x00 && h264_data[j+2] == 0x01)) {
                        next_start_code = j;
                        break;
                    }
                }
                // 提取完整的 SPS NAL 单元（包括起始码）
                cached_sps_.assign(h264_data.begin() + start_code_pos, 
                                  h264_data.begin() + next_start_code);
                RCLCPP_DEBUG(this->get_logger(), "缓存 SPS 参数集（大小: %zu 字节）", cached_sps_.size());
            }
            else if (nal == 8) {  // PPS
                // 找到下一个起始码，确定 PPS 的结束位置
                size_t next_start_code = h264_data.size();
                for (size_t j = nalu_start + 1; j + 3 < h264_data.size(); ++j) {
                    if ((h264_data[j] == 0x00 && h264_data[j+1] == 0x00 && 
                         h264_data[j+2] == 0x00 && h264_data[j+3] == 0x01) ||
                        (j + 2 < h264_data.size() && 
                         h264_data[j] == 0x00 && h264_data[j+1] == 0x00 && h264_data[j+2] == 0x01)) {
                        next_start_code = j;
                        break;
                    }
                }
                // 提取完整的 PPS NAL 单元（包括起始码）
                cached_pps_.assign(h264_data.begin() + start_code_pos, 
                                  h264_data.begin() + next_start_code);
                RCLCPP_DEBUG(this->get_logger(), "缓存 PPS 参数集（大小: %zu 字节）", cached_pps_.size());
            }
            
            // 如果找到了 SPS 和 PPS，更新状态
            if (!cached_sps_.empty() && !cached_pps_.empty()) {
                has_cached_sps_pps_ = true;
                RCLCPP_DEBUG(this->get_logger(), "SPS/PPS 缓存完成，已准备好用于 IDR 帧解码");
            }
        }
    }
}

std::vector<uint8_t> SensorDataRecorder::insertSPSPPSBeforeIDR(const std::vector<uint8_t>& h264_data)
{
    std::lock_guard<std::mutex> lock(sps_pps_mutex_);
    
    if (!has_cached_sps_pps_ || cached_sps_.empty() || cached_pps_.empty()) {
        // 如果没有缓存的 SPS/PPS，返回原始数据
        return h264_data;
    }
    
    // 查找第一个 IDR 帧的位置
    size_t first_idr_pos = 0;
    bool found_idr = false;
    
    for (size_t i = 0; i + 4 < h264_data.size(); ++i) {
        // 检查4字节起始码
        if (h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && 
            h264_data[i+2] == 0x00 && h264_data[i+3] == 0x01) {
            if (i + 4 < h264_data.size()) {
                uint8_t nal = h264_data[i + 4] & 0x1F;
                if (nal == 5) {  // IDR 帧
                    first_idr_pos = i;
                    found_idr = true;
                    break;
                }
            }
        }
        // 检查3字节起始码
        else if (i + 3 < h264_data.size() && 
                 h264_data[i] == 0x00 && h264_data[i+1] == 0x00 && h264_data[i+2] == 0x01) {
            uint8_t nal = h264_data[i+3] & 0x1F;
            if (nal == 5) {  // IDR 帧
                first_idr_pos = i;
                found_idr = true;
                break;
            }
        }
    }
    
    if (!found_idr) {
        // 如果没有找到 IDR 帧，返回原始数据
        return h264_data;
    }
    
    // 检查 IDR 帧之前是否已经包含 SPS/PPS
    std::vector<uint8_t> prefix(h264_data.begin(), h264_data.begin() + first_idr_pos);
    if (containsSPSPPS(prefix)) {
        // 已经包含 SPS/PPS，不需要插入
        RCLCPP_DEBUG(this->get_logger(), "IDR 帧前已包含 SPS/PPS，无需插入");
        return h264_data;
    }
    
    // 在 IDR 帧之前插入 SPS 和 PPS
    std::vector<uint8_t> result;
    result.reserve(h264_data.size() + cached_sps_.size() + cached_pps_.size());
    
    // 添加 IDR 帧之前的数据（如果有）
    if (first_idr_pos > 0) {
        result.insert(result.end(), h264_data.begin(), h264_data.begin() + first_idr_pos);
    }
    
    // 插入 SPS 和 PPS
    result.insert(result.end(), cached_sps_.begin(), cached_sps_.end());
    result.insert(result.end(), cached_pps_.begin(), cached_pps_.end());
    
    RCLCPP_DEBUG(this->get_logger(), "在 IDR 帧前插入 SPS/PPS：SPS=%zu 字节，PPS=%zu 字节", 
               cached_sps_.size(), cached_pps_.size());
    
    // 添加 IDR 帧及其后续数据
    result.insert(result.end(), h264_data.begin() + first_idr_pos, h264_data.end());
    
    return result;
}

void SensorDataRecorder::onDeviceBindStatus(const interface::msg::VehicleBindStatus::SharedPtr msg) {
    if (!msg) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        app_bind_page_state_ = msg->status;
        privacy_policy_ = msg->privacy_policy;

        RCLCPP_INFO(rclcpp::get_logger("SensorDataRecorder"), "onDeviceBindStatus: status=%d ,privacy_policy=%s", 
                                                                            app_bind_page_state_, privacy_policy_.c_str());
    }
}


bool SensorDataRecorder::ensureDirExists(const std::string& path) {
    size_t p = path.find_last_of('/');
    if (p == std::string::npos) return true;
    std::string dir = path.substr(0, p);
    if (dir.empty()) return true;
    struct stat st;
    if (stat(dir.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    size_t pos = 0;
    for (;;) {
        pos = dir.find('/', pos + 1);
        std::string sub = pos == std::string::npos ? dir : dir.substr(0, pos);
        if (sub.empty()) continue;
        if (stat(sub.c_str(), &st) != 0) {
            if (mkdir(sub.c_str(), 0755) != 0) return false;
        } else if (!S_ISDIR(st.st_mode)) {
            return false;
        }
        if (pos == std::string::npos) break;
    }
    return true;
}

bool SensorDataRecorder::load() {
    std::string path = "/opt/freego/user_config.yaml";
    if (!ensureDirExists(path)) 
    {
        privacy_policy_ = "";
        app_bind_page_state_ = 0;
        return false;
    }
    std::ifstream f(path);
    if (!f.good()) {
        privacy_policy_ = "";
        app_bind_page_state_ = 0;
        return false;
    }

    try {
        YAML::Node node = YAML::Load(f);
        std::lock_guard<std::mutex> lock(mutex_);
        if (node[kAppBindPageState]) {
            int value = node[kAppBindPageState].as<int>(app_bind_page_state_);
            app_bind_page_state_ = (value == 2) ? 1 : 0;
        }
        if (node[kPrivacyPolicy]) privacy_policy_ = node[kPrivacyPolicy].as<std::string>(privacy_policy_);
        return true;
    } catch (const YAML::Exception&) {
        return false;
    }
}

} // namespace sensor::recorder

