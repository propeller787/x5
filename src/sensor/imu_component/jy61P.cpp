#include "../include/imu.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

using namespace sensor::imu;

ImuComponent::ImuComponent(const rclcpp::NodeOptions& options) 
    : rclcpp::Node("imu", options),
      device_fd_(-1),
      device_path_("/dev/ttyS3"),
      publish_rate_(200.0),
      enable_temperature_(true),
      orientation_initialized_(false) {
    
    auto logger = rclcpp::get_logger("sensor::imu");
    RCLCPP_INFO(logger, "JY61P IMU Component starting...");
    
    // 声明参数
    this->declare_parameter("imu_device_path", "/dev/ttyS3");
    this->declare_parameter("publish_rate", publish_rate_);
    this->declare_parameter("enable_temperature", enable_temperature_);
    this->declare_parameter("madgwick_beta", 0.1);
    this->declare_parameter("madgwick_sample_freq", 200.0);
    this->declare_parameter("zero_vel_linear_threshold", 0.02);
    this->declare_parameter("zero_vel_angular_threshold", 0.02);
    this->declare_parameter("zero_vel_duration", 0.5);
    this->declare_parameter("accel_variance_threshold", 0.001);
    this->declare_parameter("min_calibration_interval", 30.0);
    this->declare_parameter("gyro_bias_ema_alpha", 0.001);
    this->declare_parameter("enable_zero_vel_calib", true);
    this->declare_parameter("chassis_twist_topic", std::string("/wheel/twist"));
    
    // 获取参数
    device_path_ = this->get_parameter("imu_device_path").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    enable_temperature_ = this->get_parameter("enable_temperature").as_bool();
    madgwick_beta_ = static_cast<float>(this->get_parameter("madgwick_beta").as_double());
    madgwick_sample_freq_ = static_cast<float>(this->get_parameter("madgwick_sample_freq").as_double());
    zero_vel_linear_threshold_ = this->get_parameter("zero_vel_linear_threshold").as_double();
    zero_vel_angular_threshold_ = this->get_parameter("zero_vel_angular_threshold").as_double();
    zero_vel_duration_ = this->get_parameter("zero_vel_duration").as_double();
    accel_variance_threshold_ = this->get_parameter("accel_variance_threshold").as_double();
    min_calibration_interval_ = this->get_parameter("min_calibration_interval").as_double();
    gyro_bias_ema_alpha_ = this->get_parameter("gyro_bias_ema_alpha").as_double();
    enable_zero_vel_calib_ = this->get_parameter("enable_zero_vel_calib").as_bool();
    chassis_twist_topic_ = this->get_parameter("chassis_twist_topic").as_string();
    
    RCLCPP_INFO(logger, "Device path: %s", device_path_.c_str());
    RCLCPP_INFO(logger, "Publish rate: %.1f Hz", publish_rate_);
    RCLCPP_INFO(logger, "Temperature enabled: %s", enable_temperature_ ? "true" : "false");
    
    // 设置设备
    if (!setupDevice()) {
        return;
    }
    
    // 创建发布器
    imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(imuTopic, 10);
    if (enable_temperature_) {
        temp_publisher_ = this->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
    }
    
    // 创建定时器
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
    timer_ = this->create_wall_timer(timer_period, std::bind(&ImuComponent::onTimer, this));

    // 订阅底盘速度反馈（获取速度信息用于零速检测）
    chassis_twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        chassis_twist_topic_, 10,
        std::bind(&ImuComponent::chassisTwistCallback, this, std::placeholders::_1));

    RCLCPP_INFO(logger, "JY61 IMU Component initialized successfully!");
}

ImuComponent::~ImuComponent() {
    if (device_fd_ >= 0) {
        close(device_fd_);
    }
}

bool ImuComponent::setupDevice() {
    auto logger = rclcpp::get_logger("sensor::imu");
    
    device_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Failed to open JY61 device: %s - %s", device_path_.c_str(), strerror(errno));
        return false;
    }
    
    struct termios cfg;
    tcgetattr(device_fd_, &cfg);
    bzero(&cfg, sizeof(cfg));
    
    // 设置波特率为9600 (JY61P默认波特率)
    speed_t baud = B9600;
    cfsetispeed(&cfg, baud);
    cfsetospeed(&cfg, baud);
    
    // 8数据位, 无奇偶校验, 1停止位
    cfg.c_cflag = CS8 | CREAD | CLOCAL;
    cfg.c_iflag = IGNPAR;
    cfg.c_oflag = 0;
    cfg.c_lflag = 0;
    
    cfg.c_cc[VTIME] = 0;
    cfg.c_cc[VMIN] = 0;
    
    if (tcsetattr(device_fd_, TCSANOW, &cfg) != 0) {
        RCLCPP_ERROR(logger, "Failed to set JY61 serial attributes: %s", strerror(errno));
        close(device_fd_);
        device_fd_ = -1;
        return false;
    }
    tcflush(device_fd_, TCIOFLUSH);
    {// 向JY61P发送配置命令 解锁
        uint8_t config_cmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
        write(device_fd_, config_cmd, sizeof(config_cmd));
    }
    usleep(100);
    {// 修改频率
        uint8_t config_cmd[] = {0xFF, 0xAA, 0x03, 0x0B, 0x00};
        write(device_fd_, config_cmd, sizeof(config_cmd));
    }
    tcflush(device_fd_, TCIOFLUSH);


    RCLCPP_INFO(logger, "JY61P serial port initialized successfully");
    return true;
}

void ImuComponent::onTimer() {
    readImuData();

    // 维护加速度幅值历史（方差计算用，使用 jy61p 数据）
    double accel_mag = std::sqrt(
        imu_values_jy61p_.accel_x * imu_values_jy61p_.accel_x +
        imu_values_jy61p_.accel_y * imu_values_jy61p_.accel_y +
        imu_values_jy61p_.accel_z * imu_values_jy61p_.accel_z);
    accel_mag_history_.push_back(accel_mag);
    if (accel_mag_history_.size() > kAccelWindowSize) {
        accel_mag_history_.pop_front();
    }

    // 零速检测与陀螺仪bias校准
    if (enable_zero_vel_calib_) {
        bool zero_vel = detectZeroVelocity();

        if (zero_vel && !in_zero_vel_) {
            RCLCPP_INFO(rclcpp::get_logger("sensor::imu"),
                "Zero-velocity ENTER | bias x=%.4f y=%.4f z=%.4f deg/s",
                gyro_x_bias_, gyro_y_bias_, gyro_z_bias_);
        } else if (!zero_vel && in_zero_vel_) {
            RCLCPP_INFO(rclcpp::get_logger("sensor::imu"),
                "Zero-velocity EXIT  | bias x=%.4f y=%.4f z=%.4f deg/s",
                gyro_x_bias_, gyro_y_bias_, gyro_z_bias_);
        }
        in_zero_vel_ = zero_vel;

        if (zero_vel) {
            calibrateGyroBias();
        }
    }

    publishImuData();
}

void ImuComponent::readImuData() {
    auto logger = rclcpp::get_logger("sensor::imu");
    
    uint8_t buf[11];
    uint8_t c;
    ssize_t n;
    
    // 等待帧头 0x55
    do {
        n = read(device_fd_, &c, 1);
        if (n <= 0) {
            return;
        }
    } while (c != 0x55);
    
    // 读取数据类型
    n = read(device_fd_, &buf[0], 1);
    if (n != 1) return;
    
    uint8_t type = buf[0];
    
    // 处理 0x51, 0x52, 0x53 0x59
    if (type != 0x51 && type != 0x52 && type != 0x53 && type != 0x59) {
        return;
    }
    
    // 读取剩余数据
    ssize_t total_read = 0;
    while (total_read < 9) {
        n = read(device_fd_, buf + 1 + total_read, 9 - total_read);
        if (n > 0) {
            total_read += n;
        } else if (n == 0) {
            usleep(100);
        } else {
            return;
        }
        // 超时保护
        if (total_read < 9 && total_read < 100) {
            // 继续等待
        } else if (total_read < 9) {
            tcflush(device_fd_, TCIFLUSH);
            return;
        }
    }
    
    if (total_read < 9) {
        tcflush(device_fd_, TCIFLUSH);
        return;
    }
    
    // 校验和验证
    uint8_t expected_checksum = buf[9];
    uint8_t actual_checksum = 0x55 + buf[0];
    for (int i = 1; i < 9; i++) {
        actual_checksum += buf[i];
    }
    
    if (expected_checksum != actual_checksum) {
        return;
    }
    
    // 解析数据帧，直接存入 imu_values_jy61p_ 结构
    switch (type) {
        case 0x51: {
            // 加速度计数据 (加速度原始值)
            int16_t ax = (buf[2] << 8) | buf[1];
            int16_t ay = (buf[4] << 8) | buf[3];
            int16_t az = (buf[6] << 8) | buf[5];
            int16_t temp = (buf[8] << 8) | buf[7];
            
            // 转换为g (16g量程) 并存入 imu_values_jy61p_
            imu_values_jy61p_.accel_x = ax / 32768.0f * 16.0f;
            imu_values_jy61p_.accel_y = ay / 32768.0f * 16.0f;
            imu_values_jy61p_.accel_z = az / 32768.0f * 16.0f;
            imu_values_jy61p_.temperature = temp / 100.0f;
            
            RCLCPP_INFO(logger, "加速度: X=%.3f g, Y=%.3f g, Z=%.3f g | 温度=%.2f °C",
                        imu_values_jy61p_.accel_x, imu_values_jy61p_.accel_y, 
                        imu_values_jy61p_.accel_z, imu_values_jy61p_.temperature);
            break;
        }
        case 0x52: {
            // 陀螺仪数据 (角速度原始值)
            int16_t gx = (buf[2] << 8) | buf[1];
            int16_t gy = (buf[4] << 8) | buf[3];
            int16_t gz = (buf[6] << 8) | buf[5];
            
            // 转换为°/s (2000°/s量程) 并存入 imu_values_jy61p_
            imu_values_jy61p_.gyro_x = gx / 32768.0f * 2000.0f;
            imu_values_jy61p_.gyro_y = gy / 32768.0f * 2000.0f;
            imu_values_jy61p_.gyro_z = gz / 32768.0f * 2000.0f;
            
            RCLCPP_INFO(logger, "角速度: X=%.3f °/s, Y=%.3f °/s, Z=%.3f °/s",
                        imu_values_jy61p_.gyro_x, imu_values_jy61p_.gyro_y, 
                        imu_values_jy61p_.gyro_z);
            break;
        }
        case 0x53: {
            // 欧拉角度数据 - Roll, Pitch, Yaw (°)
            int16_t roll_raw = (buf[2] << 8) | buf[1];
            int16_t pitch_raw = (buf[4] << 8) | buf[3];
            int16_t yaw_raw = (buf[6] << 8) | buf[5];
            
            // 转换为角度值 (±180°) 并存入 imu_values_jy61p_
            imu_values_jy61p_.roll = roll_raw / 32768.0f * 180.0f;
            imu_values_jy61p_.pitch = pitch_raw / 32768.0f * 180.0f;
            imu_values_jy61p_.yaw = yaw_raw / 32768.0f * 180.0f;
            
            RCLCPP_INFO(logger, "角度: Roll=%.2f °, Pitch=%.2f °, Yaw=%.2f °",
                        imu_values_jy61p_.roll, imu_values_jy61p_.pitch, 
                        imu_values_jy61p_.yaw);
            
            orientation_initialized_ = true;
            break;
        }
        case 0x59: {
            // 四元数帧
            int16_t q0_raw = (buf[2] << 8) | buf[1];
            int16_t q1_raw = (buf[4] << 8) | buf[3];
            int16_t q2_raw = (buf[6] << 8) | buf[5];
            int16_t q3_raw = (buf[8] << 8) | buf[7];
            
            // 转换为四元数并存入 imu_values_jy61p_
            imu_values_jy61p_.q0 = q0_raw / 32768.0f;
            imu_values_jy61p_.q1 = q1_raw / 32768.0f;
            imu_values_jy61p_.q2 = q2_raw / 32768.0f;
            imu_values_jy61p_.q3 = q3_raw / 32768.0f;
            
            RCLCPP_INFO(logger, "四元数: q0=%.4f, q1=%.4f, q2=%.4f, q3=%.4f",
                        imu_values_jy61p_.q0, imu_values_jy61p_.q1, 
                        imu_values_jy61p_.q2, imu_values_jy61p_.q3);
            break;
        }
        default:
            break;
    }
}

void ImuComponent::publishImuData() {
    auto now = this->now();
    
    // 发布 IMU 数据
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";
    
    // 角速度 (rad/s) - 从 imu_values_jy61p_ 获取并转换，减去零速校准bias
    const double deg2rad = M_PI / 180.0;
    imu_msg.angular_velocity.x = (imu_values_jy61p_.gyro_x - gyro_x_bias_) * deg2rad;
    imu_msg.angular_velocity.y = (imu_values_jy61p_.gyro_y - gyro_y_bias_) * deg2rad;
    imu_msg.angular_velocity.z = (imu_values_jy61p_.gyro_z - gyro_z_bias_) * deg2rad;
    
    // 线性加速度 (m/s²) - 从 imu_values_jy61p_ 获取并转换
    imu_msg.linear_acceleration.x = imu_values_jy61p_.accel_x * 9.81;
    imu_msg.linear_acceleration.y = imu_values_jy61p_.accel_y * 9.81;
    imu_msg.linear_acceleration.z = imu_values_jy61p_.accel_z * 9.81;
    
    // 四元数姿态 - 从 imu_values_jy61p_ 获取
    imu_msg.orientation.x = imu_values_jy61p_.q1;
    imu_msg.orientation.y = imu_values_jy61p_.q2;
    imu_msg.orientation.z = imu_values_jy61p_.q3;
    imu_msg.orientation.w = imu_values_jy61p_.q0;
    
    // 设置协方差矩阵
    double gyro_variance = 0.01;
    double accel_variance = 0.1;
    double orientation_variance = 0.1;
    
    imu_msg.angular_velocity_covariance[0] = gyro_variance;
    imu_msg.angular_velocity_covariance[4] = gyro_variance;
    imu_msg.angular_velocity_covariance[8] = gyro_variance;
    
    imu_msg.linear_acceleration_covariance[0] = accel_variance;
    imu_msg.linear_acceleration_covariance[4] = accel_variance;
    imu_msg.linear_acceleration_covariance[8] = accel_variance;
    
    imu_msg.orientation_covariance[0] = orientation_variance;
    imu_msg.orientation_covariance[4] = orientation_variance;
    imu_msg.orientation_covariance[8] = orientation_variance;
    
    imu_publisher_->publish(imu_msg);
    
    // 发布温度数据
    if (enable_temperature_ && temp_publisher_) {
        auto temp_msg = sensor_msgs::msg::Temperature();
        temp_msg.header.stamp = now;
        temp_msg.header.frame_id = "imu_link";
        temp_msg.temperature = imu_values_jy61p_.temperature;
        temp_msg.variance = 0.1;
        
        temp_publisher_->publish(temp_msg);
    }
}

bool ImuComponent::isValidImuData() {
    // 检查NaN和无穷大
    auto isInvalid = [](float val) -> bool {
        return std::isnan(val) || std::isinf(val) || 
               std::abs(val) > 10000.0f;
    };
    
    // 检查所有IMU值
    if (isInvalid(imu_values_jy61p_.gyro_x) || isInvalid(imu_values_jy61p_.gyro_y) || 
        isInvalid(imu_values_jy61p_.gyro_z) || isInvalid(imu_values_jy61p_.accel_x) || 
        isInvalid(imu_values_jy61p_.accel_y) || isInvalid(imu_values_jy61p_.accel_z)) {
        return false;
    }
    
    // 检查角速度是否在合理范围内（±2000°/s）
    if (std::abs(imu_values_jy61p_.gyro_x) > 2000.0f || 
        std::abs(imu_values_jy61p_.gyro_y) > 2000.0f || 
        std::abs(imu_values_jy61p_.gyro_z) > 2000.0f) {
        return false;
    }
    
    // 检查加速度是否在合理范围内（±16g）
    if (std::abs(imu_values_jy61p_.accel_x) > 16.0f ||
        std::abs(imu_values_jy61p_.accel_y) > 16.0f ||
        std::abs(imu_values_jy61p_.accel_z) > 16.0f) {
        return false;
    }

    return true;
}

void ImuComponent::chassisTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    latest_wheel_vx_ = msg->twist.linear.x;
    latest_wheel_vyaw_ = msg->twist.angular.z;
}

bool ImuComponent::detectZeroVelocity() {
    auto now = this->now();

    bool speed_below_threshold =
        std::abs(latest_wheel_vx_) < zero_vel_linear_threshold_ &&
        std::abs(latest_wheel_vyaw_) < zero_vel_angular_threshold_;
    if (speed_below_threshold) {
        if (!stationary_candidate_) {
            stationary_candidate_ = true;
            stationary_start_time_ = now;
        }
    } else {
        stationary_candidate_ = false;
        return false;
    }

    double stationary_duration = (now - stationary_start_time_).seconds();
    if (stationary_duration < zero_vel_duration_) {
        return false;
    }

    if (accel_mag_history_.size() < kAccelWindowSize) {
        return false;
    }

    double mean = 0.0;
    for (double v : accel_mag_history_) {
        mean += v;
    }
    mean /= static_cast<double>(accel_mag_history_.size());

    double variance = 0.0;
    for (double v : accel_mag_history_) {
        double d = v - mean;
        variance += d * d;
    }
    variance /= static_cast<double>(accel_mag_history_.size());

    if (variance > accel_variance_threshold_) {
        return false;
    }

    if ((now - last_calibration_time_).seconds() < min_calibration_interval_) {
        return false;
    }

    return true;
}

void ImuComponent::calibrateGyroBias() {
    const double alpha = gyro_bias_ema_alpha_;
    gyro_x_bias_ = alpha * imu_values_jy61p_.gyro_x + (1.0 - alpha) * gyro_x_bias_;
    gyro_y_bias_ = alpha * imu_values_jy61p_.gyro_y + (1.0 - alpha) * gyro_y_bias_;
    gyro_z_bias_ = alpha * imu_values_jy61p_.gyro_z + (1.0 - alpha) * gyro_z_bias_;
    last_calibration_time_ = this->now();
}