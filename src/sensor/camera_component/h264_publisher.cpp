#include "h264_publisher.h"
#include <cerrno>
#include <rclcpp/rclcpp.hpp>
#include <utils/topic.h>
#include "../../common/utils/error_event.h"
#include "../global.h"

namespace fs = std::filesystem;

H264UVC::H264UVC(const std::string& dev_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("h264_publisher", options)
{
    device_name_ = dev_name;
    
    // 声明和读取参数
    width_ = this->declare_parameter("width", 1920);
    height_ = this->declare_parameter("height", 1080);
    fps_ = this->declare_parameter("fps", 30);
    publish_fps_ = this->declare_parameter("publish_fps", 30);
    // 使用与 sensor_data_recorder 相同的存储目录参数
    storage_directory_ = this->declare_parameter("record.directory", "/opt/freego/bags");
    segment_duration_sec_ = this->declare_parameter("segment_duration_sec", 60);
    
    // 确保存储目录存在
    try {
        if (!fs::exists(storage_directory_)) {
            fs::create_directories(storage_directory_);
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create storage directory: %s", e.what());
    }
    
    last_file_time_ = std::chrono::steady_clock::now();
    
    // 创建QoS配置对象,设置历史记录深度为1
    rclcpp::QoS qos(1);
    // 设置为best_effort模式（适合视频流）
    qos.best_effort();
    // 创建H264视频流发布者
    ffmpeg_publisher_ = this->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
        cameraH264Topic, qos);
    
    // 创建同步录制服务客户端
    start_recording_client_ = this->create_client<interface::srv::StartRecordingWithPath>(
        "/sensor/recorder/start_with_path");
    stop_recording_client_ = this->create_client<std_srvs::srv::Empty>(
        "/sensor/recorder/stop");
    
    if (open_device()) {
        if (!init_device()) {
            RCLCPP_WARN(this->get_logger(),
                        "H264 init failed (format/buffers); capture disabled");
            close(fd_);
            fd_ = -1;
        } else {
            set_framerate();
            if (!start_streaming()) {
                RCLCPP_WARN(this->get_logger(),
                            "H264 STREAMON failed; capture disabled");
                uninit_device();
                close(fd_);
                fd_ = -1;
            } else {
                capture_thread_ = std::thread(&H264UVC::capture_loop, this);
            }
        }
    } else {
        RCLCPP_WARN(this->get_logger(),
                    "H264 publisher started without camera; capture thread not started");
    }
}

H264UVC::~H264UVC()
{
    running_ = false;
    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }
    
    stop_streaming();
    uninit_device();
    if (fd_ >= 0) {
        close(fd_);
    }
    
    // 关闭当前文件流并停止同步录制
    if (current_file_stream_.is_open()) {
        current_file_stream_.close();
        
        // 停止同步录制 IMU、GPS、CAN 数据
        if (stop_recording_client_ && stop_recording_client_->wait_for_service(std::chrono::seconds(1))) {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = stop_recording_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Stopped synchronized recording on shutdown");
        }
    }
}

bool H264UVC::open_device()
{
    fd_ = open(device_name_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video device: %s", device_name_.c_str());
        // 发送未检测到相机的错误码
        auto* global = sensor::Global::getInstance();
        if (global) {
            interface::msg::HexData hex_msg;
            hex_msg.hex_value = encodeErrorLevel(CAM1_NOT_FOUND, ErrorEventLevel::OCCUR);
            global->sendHexData(hex_msg);
            RCLCPP_WARN(this->get_logger(), "Sent camera not found error code: 0x%08X", hex_msg.hex_value);
        }
        return false;
    }
    return true;
}

bool H264UVC::init_device()
{
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set format (H264 %dx%d): %s",
                     width_, height_, strerror(errno));
        return false;
    }

    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        RCLCPP_ERROR(this->get_logger(), "VIDIOC_REQBUFS failed: %s", strerror(errno));
        return false;
    }

    buffers_.resize(req.count);

    for (size_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buf{};
        buf.type = req.type;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = static_cast<__u32>(i);

        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_QUERYBUF %zu failed: %s", i, strerror(errno));
            for (size_t j = 0; j < i; ++j) {
                if (buffers_[j].start != MAP_FAILED && buffers_[j].start != nullptr) {
                    munmap(buffers_[j].start, buffers_[j].length);
                    buffers_[j].start = nullptr;
                }
            }
            buffers_.clear();
            return false;
        }

        buffers_[i].length = buf.length;
        buffers_[i].start =
            mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);

        if (buffers_[i].start == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "mmap buffer %zu failed: %s", i, strerror(errno));
            for (size_t j = 0; j < i; ++j) {
                if (buffers_[j].start != MAP_FAILED && buffers_[j].start != nullptr) {
                    munmap(buffers_[j].start, buffers_[j].length);
                    buffers_[j].start = nullptr;
                }
            }
            buffers_.clear();
            return false;
        }

        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "VIDIOC_QBUF %zu failed: %s", i, strerror(errno));
            for (size_t j = 0; j <= i; ++j) {
                if (buffers_[j].start != MAP_FAILED && buffers_[j].start != nullptr) {
                    munmap(buffers_[j].start, buffers_[j].length);
                    buffers_[j].start = nullptr;
                }
            }
            buffers_.clear();
            return false;
        }
    }
    return true;
}

void H264UVC::set_framerate()
{
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator   = 1;
    parm.parm.capture.timeperframe.denominator = fps_;
    ioctl(fd_, VIDIOC_S_PARM, &parm);
}

bool H264UVC::start_streaming()
{
    if (fd_ < 0) {
        return false;
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start streaming: %s", strerror(errno));
        return false;
    }
    is_capture_ = true;
    return true;
}

void H264UVC::stop_streaming()
{
    if (fd_ < 0 || !is_capture_) {
        return;
    }
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd_, VIDIOC_STREAMOFF, &type);
    is_capture_ = false;
}

void H264UVC::uninit_device()
{
    for (auto& b : buffers_) {
        if (b.start != MAP_FAILED) {
            munmap(b.start, b.length);
        }
    }
    buffers_.clear();
}

void H264UVC::capture_loop()
{
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    auto last_file_time = std::chrono::steady_clock::now();

    while (running_ && rclcpp::ok()) {

      if (poll(&pfd, 1, 1000) <= 0)
        continue;

      v4l2_buffer buf{};
      buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0)
        continue;

      //publish_packet(buf);

      const uint8_t* data_ptr = static_cast<const uint8_t*>(buffers_[buf.index].start);
      size_t data_size = buf.bytesused;

      // Single-pass NAL classification for IDR / SPS / PPS
      uint8_t nal_flags = classify_nal(data_ptr, data_size);
      bool is_key_frame = (nal_flags & NAL_KEYFRAME) != 0;

      // Check elapsed time
      auto current_time = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        current_time - last_file_time).count();

      // Determine if we need to switch file:
      // 1. Time limit reached AND current frame is a keyframe
      // 2. OR file is not open AND current frame is a keyframe (first start)
      if (is_key_frame) {
        if (!current_file_stream_.is_open() || elapsed >= segment_duration_sec_) {
           // 使用当前帧的时间作为第一帧时间，确保文件名时间戳对应第一帧
           auto first_frame_time = std::chrono::system_clock::now();
           open_new_file(first_frame_time);
           last_file_time = current_time;
        }
      }

      // Write data if file is open
      if (current_file_stream_.is_open()) {
        current_file_stream_.write(reinterpret_cast<const char*>(data_ptr), data_size);
      }

      ioctl(fd_, VIDIOC_QBUF, &buf);
    }
}

uint8_t H264UVC::classify_nal(const uint8_t* data, size_t size)
{
    uint8_t flags = 0;
    for (size_t i = 0; i + 3 < size; ++i) {
        // 检查3字节起始码 0x000001
        if (data[i] == 0x00 && data[i+1] == 0x00 && data[i+2] == 0x01) {
            if (i + 3 < size) {
                uint8_t nal = data[i + 3] & 0x1F;
                if (nal == 5)      flags |= NAL_IDR;
                else if (nal == 7) flags |= NAL_SPS;
                else if (nal == 8) flags |= NAL_PPS;
            }
        }
        // 检查4字节起始码 0x00000001
        else if (i + 4 < size &&
                 data[i] == 0x00 && data[i+1] == 0x00 &&
                 data[i+2] == 0x00 && data[i+3] == 0x01) {
            if (i + 4 < size) {
                uint8_t nal = data[i + 4] & 0x1F;
                if (nal == 5)      flags |= NAL_IDR;
                else if (nal == 7) flags |= NAL_SPS;
                else if (nal == 8) flags |= NAL_PPS;
            }
            // 所有3种类型都找到后可提前退出
            if (flags == (NAL_IDR | NAL_SPS | NAL_PPS)) return flags;
        }
        // 两个都找到（IDR或SPS+PPS都算关键帧条件满足）则提前退出
        if (flags & NAL_IDR && flags & NAL_SPS && flags & NAL_PPS) return flags;
    }
    return flags;
}

void H264UVC::publish_packet(
    const uint8_t* data,
    size_t size,
    bool key,
    uint64_t pts,
    uint64_t sequence)
{
    auto msg = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";
    msg->encoding = "h264";
    msg->data.assign(data, data + size);
    msg->pts = pts;
    
    // 编码 flags：
    // - bit 0: key frame flag (1 = key frame, 0 = non-key frame)
    // - bits 1-31: sequence number (用于检测丢帧)
    // 注意：sequence 限制在 31 位以内 (0x7FFFFFFF)，避免溢出
    msg->flags = (key ? 1 : 0) | ((sequence & 0x7FFFFFFF) << 1);
    
    ffmpeg_publisher_->publish(std::move(msg));
}

void H264UVC::startCapture()
{
    if (!is_capture_) {
        start_streaming();
    }
}

void H264UVC::stopCapture()
{
    stop_streaming();
}

void H264UVC::setBackupCar(bool is_backup_car)
{
    is_backup_car_ = is_backup_car;
}

std::string H264UVC::generate_sensor_folder_path(std::chrono::system_clock::time_point time_point)
{
    // 生成 sensor_YYYYMMDD_HHMMSS 格式的文件夹名
    auto time_t = std::chrono::system_clock::to_time_t(time_point);
    std::tm* tm_info = std::localtime(&time_t);
    
    std::ostringstream oss;
    oss << storage_directory_ << "/sensor_"
        << std::setfill('0') << std::setw(4) << (tm_info->tm_year + 1900)
        << std::setw(2) << (tm_info->tm_mon + 1)
        << std::setw(2) << tm_info->tm_mday << "_"
        << std::setw(2) << tm_info->tm_hour
        << std::setw(2) << tm_info->tm_min
        << std::setw(2) << tm_info->tm_sec;
    
    return oss.str();
}

std::string H264UVC::get_current_sensor_folder()
{
    return current_sensor_folder_;
}

void H264UVC::open_new_file(std::chrono::system_clock::time_point first_frame_time)
{
    std::lock_guard<std::mutex> lock(file_mutex_);
    
    // 关闭当前文件并停止同步录制
    if (current_file_stream_.is_open()) {
        current_file_stream_.close();
        RCLCPP_INFO(this->get_logger(), "Finished recording segment.");
        
        // 停止同步录制 IMU、GPS、CAN 数据
        if (stop_recording_client_ && stop_recording_client_->wait_for_service(std::chrono::seconds(1))) {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result = stop_recording_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Stopped synchronized recording of IMU, GPS, CAN data");
        } else {
            RCLCPP_WARN(this->get_logger(), "Stop recording service not available");
        }
    }
    
    // 为每个 h264 文件生成新的文件夹路径（格式：sensor_YYYYMMDD_HHMMSS）
    current_sensor_folder_ = generate_sensor_folder_path(first_frame_time);
    
    // 创建新的 sensor 文件夹
    try {
        if (!fs::exists(current_sensor_folder_)) {
            fs::create_directories(current_sensor_folder_);
            RCLCPP_INFO(this->get_logger(), "Created sensor folder: %s", current_sensor_folder_.c_str());
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to create sensor folder: %s", e.what());
        return;
    }
    
    // Generate filename with timestamp using first frame time
    // 使用第一帧的时间戳作为文件名，确保文件名时间对应第一帧
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        first_frame_time.time_since_epoch()).count();
    
    std::ostringstream filename_ss;
    filename_ss << ms << ".h264";
    fs::path filepath = fs::path(current_sensor_folder_) / filename_ss.str();
    
    // 打开新文件
    current_file_stream_.open(filepath.string(), std::ios::binary);
    
    if (current_file_stream_.is_open()) {
        RCLCPP_INFO(this->get_logger(), "Started new recording segment (first frame time): %s", filepath.c_str());
        
        // 开始同步录制 IMU、GPS、CAN 数据到同一个文件夹
        if (start_recording_client_ && start_recording_client_->wait_for_service(std::chrono::seconds(1))) {
            auto request = std::make_shared<interface::srv::StartRecordingWithPath::Request>();
            request->folder_path = current_sensor_folder_;
            auto result = start_recording_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Started synchronized recording of IMU, GPS, CAN data in folder: %s", 
                       current_sensor_folder_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Start recording service not available, continuing without synchronized recording");
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to create file: %s", filepath.c_str());
    }
}
