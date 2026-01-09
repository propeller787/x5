#include "sensor_data_recorder.hpp"
#include <std_srvs/srv/empty.hpp>
#include <iomanip>
#include <sstream>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cstdio>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <fstream>

namespace sensor::recorder {

SensorDataRecorder::SensorDataRecorder(const rclcpp::NodeOptions& options)
    : rclcpp::Node("sensor_data_recorder", options)
{
    auto logger = rclcpp::get_logger("sensor_data_recorder");
    RCLCPP_INFO(logger, "Sensor Data Recorder starting...");
    
    // 加载配置
    loadConfig();
    
    // 创建录制目录
    createBagDirectory();
    rclcpp::QoS qos(1);
    // 设置为best_effort模式（适合视频流）
    qos.best_effort();
    // 创建订阅器
    camera_sub_ = this->create_subscription<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
        cameraH264Topic, qos,
        std::bind(&SensorDataRecorder::cameraCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imuTopic, 10,
        std::bind(&SensorDataRecorder::imuCallback, this, std::placeholders::_1));
    
    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        gpsTopic, 10,
        std::bind(&SensorDataRecorder::gpsCallback, this, std::placeholders::_1));
    
    // 订阅CAN数据 - 需要从handle_can_adapter发布的话题订阅
    // 这里假设CAN数据会发布到一个话题，如果没有，需要先创建发布器
    can_sub_ = this->create_subscription<can_msgs::msg::Frame>(
        "/can/frames", 100,  // 使用较大的队列深度，因为CAN数据频率可能很高
        std::bind(&SensorDataRecorder::canCallback, this, std::placeholders::_1));
    
    // 订阅雷达数据
    lidar_right_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightLaserTopic, 10,
        std::bind(&SensorDataRecorder::lidarRightCallback, this, std::placeholders::_1));
    
    lidar_left_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftLaserTopic, 10,
        std::bind(&SensorDataRecorder::lidarLeftCallback, this, std::placeholders::_1));
    
    lidar_left_tilt_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftTiltLaserTopic, 10,
        std::bind(&SensorDataRecorder::lidarLeftTiltCallback, this, std::placeholders::_1));
    
    lidar_right_tilt_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightTiltLaserTopic, 10,
        std::bind(&SensorDataRecorder::lidarRightTiltCallback, this, std::placeholders::_1));
    
    lidar_back_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        backLaserTopic, 10,
        std::bind(&SensorDataRecorder::lidarBackCallback, this, std::placeholders::_1));
    
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
    
    // 创建服务用于控制录制
    start_record_service_ = this->create_service<std_srvs::srv::Empty>(
        "/sensor/recorder/start",
        std::bind(&SensorDataRecorder::startRecordServiceCallback, this,
                 std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    
    stop_record_service_ = this->create_service<std_srvs::srv::Empty>(
        "/sensor/recorder/stop",
        std::bind(&SensorDataRecorder::stopRecordServiceCallback, this,
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
    RCLCPP_INFO(logger, "Max bag duration: %d minutes", max_bag_duration_minutes_);
    RCLCPP_INFO(logger, "Max bag size: %d MB", max_bag_size_mb_);
    RCLCPP_INFO(logger, "Lidar bag prefix: %s", lidar_bag_prefix_.c_str());
    RCLCPP_INFO(logger, "Camera verify mode: %s", verify_camera_enable_ ? "enabled" : "disabled");
}

SensorDataRecorder::~SensorDataRecorder()
{
    stopRecording();
    stopLidarRecording();
}

void SensorDataRecorder::loadConfig()
{
    // 声明并获取参数
    this->declare_parameter("record.directory", std::string("./bags"));
    this->declare_parameter("record.bag_prefix", std::string("sensor_data"));
    this->declare_parameter("record.max_duration_minutes", 1);
    this->declare_parameter("record.max_size_mb", 1024);  // 1GB
    this->declare_parameter("record.auto_start", true);
    
    // 雷达数据录制配置
    this->declare_parameter("record.lidar.bag_prefix", std::string("lidar_data"));
    this->declare_parameter("record.lidar.auto_start", true);
    
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
    auto_start_lidar_recording_ = this->get_parameter("record.lidar.auto_start").as_bool();
    
    verify_camera_enable_ = this->get_parameter("verify_camera.enable").as_bool();
    verify_image_dir_ = this->get_parameter("verify_camera.image_dir").as_string();
    verify_frame_interval_ = this->get_parameter("verify_camera.frame_interval").as_int();
    verify_frame_count_ = 0;
    
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
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::tm* tm_info = std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << record_dir_ << "/" << bag_prefix_ << "_"
        << std::setfill('0') << std::setw(4) << (tm_info->tm_year + 1900)
        << std::setw(2) << (tm_info->tm_mon + 1)
        << std::setw(2) << tm_info->tm_mday << "_"
        << std::setw(2) << tm_info->tm_hour
        << std::setw(2) << tm_info->tm_min
        << std::setw(2) << tm_info->tm_sec;
    
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
    std::lock_guard<std::mutex> lock(writer_mutex_);
    
    if (is_recording_.load()) {
        RCLCPP_WARN(this->get_logger(), "Recording is already started");
        return;
    }
    
    try {
        current_bag_path_ = generateBagPath();
        current_bag_start_time_ = std::chrono::system_clock::now();
        current_bag_size_bytes_ = 0;
        
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
        
        // Camera话题
        topic_metadata.name = cameraH264Topic;
        topic_metadata.type = "ffmpeg_image_transport_msgs/msg/FFMPEGPacket";
        topic_metadata.serialization_format = "cdr";
        bag_writer_->create_topic(topic_metadata);
        
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
        
        RCLCPP_INFO(this->get_logger(), "Started recording to: %s", current_bag_path_.c_str());
        RCLCPP_INFO(this->get_logger(), "Recording topics: %s, %s, %s, /can/frames",
                   cameraH264Topic, imuTopic, gpsTopic);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
        if (bag_writer_) {
            bag_writer_.reset();
        }
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
    if (!is_recording_.load()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(writer_mutex_);
    
    if (!bag_writer_) {
        return;
    }
    
    try {
        // 序列化消息
        rclcpp::SerializedMessage serialized_msg;
        
        if (topic_type == "ffmpeg_image_transport_msgs/msg/FFMPEGPacket") {
            const auto* msg = static_cast<const ffmpeg_image_transport_msgs::msg::FFMPEGPacket*>(msg_ptr);
            camera_serialization_.serialize_message(msg, &serialized_msg);
        } else if (topic_type == "sensor_msgs/msg/Imu") {
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
        
        // 创建bag消息
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        
        // 复制序列化数据到bag消息
        const rcl_serialized_message_t& rcl_msg = serialized_msg.get_rcl_serialized_message();
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t,
            [](rcutils_uint8_array_t* msg) {
                if (msg->buffer != nullptr) {
                    delete[] msg->buffer;
                }
                delete msg;
            }
        );
        
        // 分配并复制数据
        bag_message->serialized_data->buffer = new uint8_t[rcl_msg.buffer_length];
        bag_message->serialized_data->buffer_length = rcl_msg.buffer_length;
        bag_message->serialized_data->buffer_capacity = rcl_msg.buffer_length;
        std::memcpy(bag_message->serialized_data->buffer, rcl_msg.buffer, rcl_msg.buffer_length);
        
        // 设置时间戳
        bag_message->time_stamp = timestamp.nanoseconds();
        
        // 设置topic名称
        bag_message->topic_name = topic_name;
        
        // 写入bag
        bag_writer_->write(bag_message);
        
        // 更新当前bag文件大小
        current_bag_size_bytes_ += rcl_msg.buffer_length;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write message to bag: %s", e.what());
    }
}

void SensorDataRecorder::cameraCallback(const ffmpeg_image_transport_msgs::msg::FFMPEGPacket::SharedPtr msg)
{
    writeMessage(cameraH264Topic, "ffmpeg_image_transport_msgs/msg/FFMPEGPacket", 
                msg->header.stamp, msg.get());
    
    // 如果启用了验证模式，解码H264并保存图片
    if (verify_camera_enable_) {
        std::lock_guard<std::mutex> lock(verify_mutex_);
        
        verify_frame_count_++;
        
        // 检查当前packet是否包含IDR帧
        bool current_has_idr = false;
        if (!msg->data.empty() && hasValidH264StartCode(msg->data) && hasIDRFrame(msg->data)) {
            current_has_idr = true;
        }
        
        // 限制缓冲区大小，避免内存溢出（最大20MB）
        const size_t MAX_BUFFER_SIZE = 20 * 1024 * 1024;
        if (h264_buffer_.size() + msg->data.size() > MAX_BUFFER_SIZE) {
            RCLCPP_WARN(this->get_logger(), "H264缓冲区过大，清空缓冲区并重新开始");
            h264_buffer_.clear();
            verify_frame_count_ = 0;
        }
        
        // 累积H264数据到缓冲区
        h264_buffer_.insert(h264_buffer_.end(), msg->data.begin(), msg->data.end());
        
        // 根据帧间隔决定是否解码并保存图片
        // 优先使用包含IDR帧的当前packet，如果没有则尝试使用累积的缓冲区
        if (verify_frame_count_ >= static_cast<uint32_t>(verify_frame_interval_)) {
            bool decoded = false;
            
            // 策略1: 如果当前packet包含IDR帧，直接使用当前packet解码（更可靠）
            if (current_has_idr && !msg->data.empty()) {
                RCLCPP_DEBUG(this->get_logger(), "检测到IDR帧，使用当前packet解码（大小: %zu 字节）", 
                           msg->data.size());
                decodeAndSaveImage(msg->data, msg->header.stamp);
                decoded = true;
            }
            // 策略2: 如果累积的缓冲区包含IDR帧，使用缓冲区解码
            else if (!h264_buffer_.empty() && hasValidH264StartCode(h264_buffer_) && hasIDRFrame(h264_buffer_)) {
                RCLCPP_DEBUG(this->get_logger(), "使用累积缓冲区解码（大小: %zu 字节）", h264_buffer_.size());
                decodeAndSaveImage(h264_buffer_, msg->header.stamp);
                decoded = true;
            }
            // 策略3: 如果都不包含IDR帧，但数据有效，仍然尝试解码（可能包含SPS/PPS和普通帧）
            else if (!h264_buffer_.empty() && hasValidH264StartCode(h264_buffer_)) {
                RCLCPP_DEBUG(this->get_logger(), 
                           "未检测到IDR帧，但数据有效，尝试解码（缓冲区大小: %zu 字节）", 
                           h264_buffer_.size());
                decodeAndSaveImage(h264_buffer_, msg->header.stamp);
                decoded = true;
            }
            
            if (!decoded && !h264_buffer_.empty()) {
                RCLCPP_DEBUG(this->get_logger(), 
                           "H264数据无效，跳过解码（缓冲区大小: %zu 字节）", 
                           h264_buffer_.size());
            }
            
            // 清空缓冲区，准备下一轮累积
            h264_buffer_.clear();
            verify_frame_count_ = 0;
        }
    }
}

void SensorDataRecorder::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    writeMessage(imuTopic, "sensor_msgs/msg/Imu", msg->header.stamp, msg.get());
}

void SensorDataRecorder::gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    //writeMessage(gpsTopic, "sensor_msgs/msg/NavSatFix", msg->header.stamp, msg.get());
}

void SensorDataRecorder::canCallback(const can_msgs::msg::Frame::SharedPtr msg)
{
    writeMessage("/can/frames", "can_msgs/msg/Frame", msg->header.stamp, msg.get());
}

void SensorDataRecorder::lidarRightCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    writeLidarMessage("/scan/right", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
}

void SensorDataRecorder::lidarLeftCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    writeLidarMessage("/scan/left", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
}

void SensorDataRecorder::lidarLeftTiltCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    writeLidarMessage("/scan/left_tilt", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
}

void SensorDataRecorder::lidarRightTiltCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    writeLidarMessage("/scan/right_tilt", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
}

void SensorDataRecorder::lidarBackCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    writeLidarMessage("/scan/back", "sensor_msgs/msg/LaserScan", msg->header.stamp, msg.get());
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
        } else if (param.get_name() == "record.lidar.auto_start") {
            auto_start_lidar_recording_ = param.as_bool();
            RCLCPP_INFO(this->get_logger(), "Auto start lidar recording updated to: %s", 
                       auto_start_lidar_recording_ ? "true" : "false");
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
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start lidar recording: %s", e.what());
        if (lidar_bag_writer_) {
            lidar_bag_writer_.reset();
        }
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
    if (!is_lidar_recording_.load()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(lidar_writer_mutex_);
    
    if (!lidar_bag_writer_) {
        return;
    }
    
    try {
        // 序列化消息
        rclcpp::SerializedMessage serialized_msg;
        
        if (topic_type == "sensor_msgs/msg/LaserScan") {
            const auto* msg = static_cast<const sensor_msgs::msg::LaserScan*>(msg_ptr);
            lidar_serialization_.serialize_message(msg, &serialized_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown lidar topic type: %s", topic_type.c_str());
            return;
        }
        
        // 创建bag消息
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        
        // 复制序列化数据到bag消息
        const rcl_serialized_message_t& rcl_msg = serialized_msg.get_rcl_serialized_message();
        bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
            new rcutils_uint8_array_t,
            [](rcutils_uint8_array_t* msg) {
                if (msg->buffer != nullptr) {
                    delete[] msg->buffer;
                }
                delete msg;
            }
        );
        
        // 分配并复制数据
        bag_message->serialized_data->buffer = new uint8_t[rcl_msg.buffer_length];
        bag_message->serialized_data->buffer_length = rcl_msg.buffer_length;
        bag_message->serialized_data->buffer_capacity = rcl_msg.buffer_length;
        std::memcpy(bag_message->serialized_data->buffer, rcl_msg.buffer, rcl_msg.buffer_length);
        
        // 设置时间戳
        bag_message->time_stamp = timestamp.nanoseconds();
        
        // 设置topic名称
        bag_message->topic_name = topic_name;
        
        // 写入bag
        lidar_bag_writer_->write(bag_message);
        
        // 更新当前bag文件大小
        current_lidar_bag_size_bytes_ += rcl_msg.buffer_length;
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write lidar message to bag: %s", e.what());
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
    auto now = std::chrono::system_clock::now();
    auto duration_seconds = std::chrono::duration_cast<std::chrono::seconds>(
        now - current_lidar_bag_start_time_).count();
    int64_t max_duration_seconds = static_cast<int64_t>(max_bag_duration_minutes_) * 60;
    
    if (duration_seconds >= max_duration_seconds) {
        need_new_bag = true;
        RCLCPP_INFO(this->get_logger(), "Lidar bag file duration (%ld seconds, %.2f minutes) exceeds limit (%d minutes), creating new bag",
                   duration_seconds, duration_seconds / 60.0, max_bag_duration_minutes_);
    }
    
    // 检查大小
    int64_t max_size_bytes = static_cast<int64_t>(max_bag_size_mb_) * 1024 * 1024;
    if (current_lidar_bag_size_bytes_ >= max_size_bytes) {
        need_new_bag = true;
        RCLCPP_INFO(this->get_logger(), "Lidar bag file size (%ld bytes) exceeds limit (%d MB), creating new bag",
                   current_lidar_bag_size_bytes_, max_bag_size_mb_);
    }
    
    if (need_new_bag) {
        // 停止当前录制并开始新的
        stopLidarRecording();
        startLidarRecording();
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

} // namespace sensor::recorder

