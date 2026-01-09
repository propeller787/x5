#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>

#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>

struct Buffer {
    void* start;
    size_t length;
};

class H264UVC : public rclcpp::Node {
public:
    H264UVC(const std::string& dev_name, const rclcpp::NodeOptions& options);
    ~H264UVC();
    void startCapture();
    void stopCapture();
    void setBackupCar(bool is_backup_car);

private:
    void open_device();
    void init_device();
    void start_streaming();
    void stop_streaming();
    void uninit_device();
    void capture_loop();
    
    bool is_idr_frame(const uint8_t* data, size_t size);
    void publish_packet(const uint8_t* data, size_t size, bool key, uint64_t pts);

private:
    std::string device_name_;
    int width_ = 1920;
    int height_ = 1080;
    int fps_ = 30;
    int publish_fps_ = 15;  // 发布帧率（10, 15, 20, 30）
    double frame_skip_interval_;  // 帧跳过间隔 (30.0 / publish_fps_)
    
    int fd_ = -1;
    std::vector<Buffer> buffers_;
    std::atomic<bool> running_{true};
    std::atomic<bool> is_capture_{false};
    std::thread capture_thread_;
    
    rclcpp::Publisher<ffmpeg_image_transport_msgs::msg::FFMPEGPacket>::SharedPtr ffmpeg_publisher_;
    bool is_backup_car_ = false;
    uint64_t pts_counter_ = 0;
};