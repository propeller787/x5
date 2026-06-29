#pragma once
#include <rclcpp/rclcpp.hpp>
#include <ffmpeg_image_transport_msgs/msg/ffmpeg_packet.hpp>

#include <linux/videodev2.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <poll.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <thread>
#include <atomic>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <mutex>
#include <interface/srv/start_recording_with_path.hpp>
#include <std_srvs/srv/empty.hpp>

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
    /** @return true if device opened, false on failure (node still usable, no capture) */
    bool open_device();
    bool init_device();
    void set_framerate();
    bool start_streaming();
    void stop_streaming();
    void uninit_device();
    void capture_loop();
    
    // NAL classification bitmask: bit0=IDR, bit1=SPS, bit2=PPS
    enum NalFlags : uint8_t { NAL_IDR = 1, NAL_SPS = 2, NAL_PPS = 4, NAL_KEYFRAME = NAL_IDR | NAL_SPS | NAL_PPS };
    uint8_t classify_nal(const uint8_t* data, size_t size);
    void publish_packet(const uint8_t* data, size_t size, bool key, uint64_t pts, uint64_t sequence);
    
    // H264 文件落盘相关
    void open_new_file(std::chrono::system_clock::time_point first_frame_time);
    std::string generate_sensor_folder_path(std::chrono::system_clock::time_point time_point);
    std::string get_current_sensor_folder();

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
    uint64_t sequence_counter_ = 0;  // 序列号计数器，用于检测丢帧
    
    // H264 文件落盘相关
    std::string storage_directory_ = ".";  // 存储目录
    int segment_duration_sec_ = 60;  // 每个文件段的时长（秒）
    std::ofstream current_file_stream_;  // 当前文件流
    std::string current_sensor_folder_;  // 当前 sensor 文件夹路径
    std::chrono::steady_clock::time_point last_file_time_;  // 上次创建文件的时间
    std::mutex file_mutex_;  // 文件操作互斥锁
    
    // 同步录制服务客户端
    rclcpp::Client<interface::srv::StartRecordingWithPath>::SharedPtr start_recording_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr stop_recording_client_;
};