#include "h264_publisher.h"
#include <rclcpp/rclcpp.hpp>
#include <utils/topic.h>

H264UVC::H264UVC(const std::string& dev_name, const rclcpp::NodeOptions& options)
    : rclcpp::Node("h264_publisher", options)
{
    device_name_ = dev_name;
    
    // 声明和读取参数
    width_ = this->declare_parameter("width", 1920);
    height_ = this->declare_parameter("height", 1080);
    fps_ = this->declare_parameter("fps", 30);
    publish_fps_ = this->declare_parameter("publish_fps", 15);
    
    // 验证publish_fps是否为支持的值：10, 15, 20, 30
    if (publish_fps_ != 10 && publish_fps_ != 15 && publish_fps_ != 20 && publish_fps_ != 30) {
        RCLCPP_WARN(this->get_logger(), 
                   "Invalid publish_fps: %d, using default 15", publish_fps_);
        publish_fps_ = 15;
    }
    
    // 计算帧率控制参数
    frame_skip_interval_ = 30.0 / publish_fps_;  // 30Hz源帧率 / 目标发布帧率
    
    RCLCPP_INFO(this->get_logger(), 
               "Camera capture FPS: %dHz, Publish FPS: %dHz", fps_, publish_fps_);
    
    // 创建QoS配置对象,设置历史记录深度为1
    rclcpp::QoS qos(1);
    // 设置为best_effort模式（适合视频流）
    qos.best_effort();
    // 创建H264视频流发布者
    ffmpeg_publisher_ = this->create_publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>(
        cameraH264Topic, qos);
    
    open_device();
    init_device();
    start_streaming();
    
    capture_thread_ = std::thread(&H264UVC::capture_loop, this);
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
}

void H264UVC::open_device()
{
    fd_ = open(device_name_.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open video device: %s", device_name_.c_str());
        throw std::runtime_error("Failed to open video device");
    }
}

void H264UVC::init_device()
{
    v4l2_format fmt{};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width_;
    fmt.fmt.pix.height = height_;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_H264;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    
    if (ioctl(fd_, VIDIOC_S_FMT, &fmt) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set format");
        throw std::runtime_error("Failed to set format");
    }
    
    v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = fps_;
    if (ioctl(fd_, VIDIOC_S_PARM, &parm) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set framerate");
        throw std::runtime_error("Failed to set framerate");
    }
    
    v4l2_requestbuffers req{};
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to request buffers");
        throw std::runtime_error("Failed to request buffers");
    }
    
    buffers_.resize(req.count);
    
    for (size_t i = 0; i < buffers_.size(); ++i) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        if (ioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to query buffer");
            throw std::runtime_error("Failed to query buffer");
        }
        
        buffers_[i].length = buf.length;
        buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, fd_, buf.m.offset);
        
        if (buffers_[i].start == MAP_FAILED) {
            RCLCPP_ERROR(this->get_logger(), "Failed to mmap buffer");
            throw std::runtime_error("Failed to mmap buffer");
        }
        
        if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to queue buffer");
            throw std::runtime_error("Failed to queue buffer");
        }
    }
}

void H264UVC::start_streaming()
{
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start streaming");
        throw std::runtime_error("Failed to start streaming");
    }
    is_capture_ = true;
}

void H264UVC::stop_streaming()
{
    if (is_capture_) {
        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(fd_, VIDIOC_STREAMOFF, &type);
        is_capture_ = false;
    }
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
    double frame_accumulator = 0.0;  // 帧率控制累加器
    
    while (rclcpp::ok() && running_ && is_capture_) {
        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(fd_, VIDIOC_DQBUF, &buf) == 0) {
            uint8_t* data = static_cast<uint8_t*>(buffers_[buf.index].start);
            size_t size = buf.bytesused;
            
            bool is_key = is_idr_frame(data, size);
            
            // 帧率控制：只发送符合条件的帧
            frame_accumulator += 1.0;
            
            // IDR关键帧总是发送，以确保接收端可以正确解码
            bool should_publish = is_key;
            
            // 如果累加器达到发送间隔，也发送此帧
            if (frame_accumulator >= frame_skip_interval_) {
                should_publish = true;
                frame_accumulator -= frame_skip_interval_;
            }
            
            if (should_publish) {
                publish_packet(data, size, is_key, pts_counter_++);
            }
            
            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
                RCLCPP_ERROR(this->get_logger(), "Failed to queue buffer");
            }
        } else {
            if (errno != EAGAIN && errno != EINTR) {
                RCLCPP_WARN(this->get_logger(), "Failed to dequeue buffer: %s", strerror(errno));
            }
            usleep(1000);
        }
    }
}

bool H264UVC::is_idr_frame(const uint8_t* data, size_t size)
{
    for (size_t i = 0; i + 4 < size; ++i) {
        if (data[i] == 0x00 && data[i+1] == 0x00 &&
            data[i+2] == 0x00 && data[i+3] == 0x01) {
            uint8_t nal = data[i+4] & 0x1F;
            return nal == 5;  // IDR
        }
    }
    return false;
}

void H264UVC::publish_packet(
    const uint8_t* data,
    size_t size,
    bool key,
    uint64_t pts)
{
    auto msg = std::make_unique<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";
    msg->encoding = "h264";
    msg->data.assign(data, data + size);
    msg->pts = pts;
    msg->flags = key ? 1 : 0;
    
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
