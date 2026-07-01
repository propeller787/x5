#include "imu.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <chrono>
#include <cmath>
#include "../global.h"
#include "../../common/utils/error_event.h"
#include <fcntl.h>
#include <unistd.h>
#include <limits>

using namespace sensor::imu;

// 静态成员变量，用于跟踪错误状态，避免重复报告
static bool device_not_found_reported = false;
static bool device_disconnected_reported = false;
static std::chrono::steady_clock::time_point last_successful_read_time;
static constexpr int MAX_READ_FAILURES = 5;  // 连续读取失败次数阈值
static int consecutive_read_failures = 0;
// 错误状态跟踪：记录当前处于错误状态的错误码（0表示无错误）
static uint32_t current_error_code = 0;
// 数据无效错误状态跟踪（单独处理，因为它是瞬时性的）
static bool data_invalid_reported = false;

ImuComponent::ImuComponent(const rclcpp::NodeOptions& options) 
    : rclcpp::Node("imu", options),
      device_fd_(-1),
      device_path_("/dev/mpu6500"),
      publish_rate_(10.0),
      enable_temperature_(true),
      orientation_initialized_(false),
      madgwick_beta_(0.1f),
      madgwick_sample_freq_(static_cast<float>(publish_rate_)),
      q0_(1.0f), q1_(0.0f), q2_(0.0f), q3_(0.0f) {
    
    // 初始化四元数为单位四元数 (无旋转)
    orientation_.setRPY(0.0, 0.0, 0.0);
    
    auto logger = rclcpp::get_logger("sensor::imu");
    RCLCPP_INFO(logger, "IMU Component starting...");
    
    // 声明参数
    this->declare_parameter("imu_device_path", "/dev/mpu6500");
    this->declare_parameter("publish_rate", publish_rate_);
    this->declare_parameter("enable_temperature", enable_temperature_);
    this->declare_parameter("madgwick_beta", static_cast<double>(madgwick_beta_));
    this->declare_parameter("madgwick_sample_freq", static_cast<double>(madgwick_sample_freq_));
    this->declare_parameter("zero_vel_linear_threshold", 0.02);
    this->declare_parameter("zero_vel_angular_threshold", 0.02);
    this->declare_parameter("zero_vel_duration", 0.5);
    this->declare_parameter("accel_variance_threshold", 0.001);
    this->declare_parameter("min_calibration_interval", 30.0);
    this->declare_parameter("gyro_bias_ema_alpha", 0.001);
    this->declare_parameter("chassis_twist_topic", std::string("/wheel/twist"));
    this->declare_parameter("enable_zero_vel_calib", true);
    
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
    chassis_twist_topic_ = this->get_parameter("chassis_twist_topic").as_string();
    enable_zero_vel_calib_ = this->get_parameter("enable_zero_vel_calib").as_bool();
    
    // 确保采样频率与发布频率一致
    madgwick_sample_freq_ = static_cast<float>(publish_rate_);
    
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
    
    RCLCPP_INFO(logger, "IMU Component initialized successfully!");
}

ImuComponent::~ImuComponent() {
    if (device_fd_ >= 0) {
        close(device_fd_);
    }
}

bool ImuComponent::setupDevice() {
    auto logger = rclcpp::get_logger("sensor::imu");
    // device_path_ = "/dev/mpu6050";
    // device_fd_ = open(device_path_.c_str(), O_RDWR);
    // device_path_ = "/dev/mpu6500";
    device_fd_ = open(device_path_.c_str(), O_RDONLY);
    RCLCPP_INFO(logger, "Device path: %s", device_path_.c_str());    
    if (device_fd_ < 0) {
        RCLCPP_ERROR(logger, "Failed to open device: %s", device_path_.c_str());
        
        // 报告设备未找到错误
        if (!device_not_found_reported) {
            auto* global = sensor::Global::getInstance();
            if (global && global->node_) {
                interface::msg::HexData error_msg;
                error_msg.hex_value = encodeErrorLevel(IMU_NOT_FOUND, ErrorEventLevel::OCCUR);
                global->sendHexData(error_msg);
                RCLCPP_ERROR(logger, "Error event reported: IMU_NOT_FOUND (0x%08X)", IMU_NOT_FOUND);
                current_error_code = IMU_NOT_FOUND; // 记录错误状态
            }
            device_not_found_reported = true;
        }
        return false;
    }
    
    // 设备打开成功，重置错误状态
    device_not_found_reported = false;
    device_disconnected_reported = false;
    consecutive_read_failures = 0;
    last_successful_read_time = std::chrono::steady_clock::now();
    data_invalid_reported = false;
    
    // 如果之前处于错误状态，现在恢复，发布恢复消息
    if (current_error_code != 0) {
        auto* global = sensor::Global::getInstance();
        if (global && global->node_) {
            RCLCPP_INFO(logger, "IMU recovered from error (0x%08X)", current_error_code);
            interface::msg::HexData recover_msg;
            recover_msg.hex_value = encodeErrorLevel(current_error_code, ErrorEventLevel::RECOVER);
            global->sendHexData(recover_msg);
        }
        current_error_code = 0; // 清除错误状态
    }
    
    RCLCPP_INFO(logger, "Successfully opened device: %s", device_path_.c_str());
    return true;
}

void ImuComponent::onTimer() {
    readImuData();
    updateOrientation();

    // 维护加速度幅值历史（方差计算用）
    double accel_mag = std::sqrt(
        imu_values_.accel_x * imu_values_.accel_x +
        imu_values_.accel_y * imu_values_.accel_y +
        imu_values_.accel_z * imu_values_.accel_z);
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
    auto* global = sensor::Global::getInstance();
    
    // 参考mpu6050Demo.c，使用数组缓冲区读取7个数据
    // signed int buffer[7]; // 用于存储从MPU6050读取的7个原始数据（gx,gy,gz,ax,ay,az,temp）
    // int ret = read(device_fd_, buffer, sizeof(buffer));
    // if(ret == 0){
    //     // 提取原始ADC值，按照mpu6050Demo.c的顺序
    //     imu_data_.gyro_x = buffer[0];    // X轴陀螺仪原始ADC值
    //     imu_data_.gyro_y = buffer[1];    // Y轴陀螺仪原始ADC值
    //     imu_data_.gyro_z = buffer[2];    // Z轴陀螺仪原始ADC值
    //     imu_data_.accel_x = buffer[3];   // X轴加速度计原始ADC值
    //     imu_data_.accel_y = buffer[4];   // Y轴加速度计原始ADC值
    //     imu_data_.accel_z = buffer[5];   // Z轴加速度计原始ADC值
    //     imu_data_.temperature = buffer[6]; // 温度原始ADC值
        
    //     // 转换数据
    //     imu_values_.gyro_x = convertGyroToDegreesPerSecond(imu_data_.gyro_x);
    //     imu_values_.gyro_y = convertGyroToDegreesPerSecond(imu_data_.gyro_y);
    //     imu_values_.gyro_z = convertGyroToDegreesPerSecond(imu_data_.gyro_z);
        
    //     imu_values_.accel_x = convertAccelToG(imu_data_.accel_x);
    //     imu_values_.accel_y = convertAccelToG(imu_data_.accel_y);
    //     imu_values_.accel_z = convertAccelToG(imu_data_.accel_z);
        
    //     imu_values_.temperature = convertTempToCelsius_6050(imu_data_.temperature);
    // }

    // 读取mpu6500数据
    typedef struct {
        int16_t ax;
        int16_t ay;
        int16_t az;
        int16_t gx;
        int16_t gy;
        int16_t gz;
        int16_t temp;
    } mpu6500_data_t;
    mpu6500_data_t d;
    ssize_t n = read(device_fd_, &d, sizeof(d));
    if(n == sizeof(d)) {
        imu_data_.gyro_x = d.gx;    // X轴陀螺仪原始ADC值
        imu_data_.gyro_y = d.gy;    // Y轴陀螺仪原始ADC值
        imu_data_.gyro_z = d.gz;    // Z轴陀螺仪原始ADC值
        imu_data_.accel_x = d.ax;   // X轴加速度计原始ADC值
        imu_data_.accel_y = d.ay;   // Y轴加速度计原始ADC值
        imu_data_.accel_z = d.az;   // Z轴加速度计原始ADC值
        imu_data_.temperature = d.temp; // 温度原始ADC值
        
        // 转换数据
        imu_values_.gyro_x = convertGyroToDegreesPerSecond(imu_data_.gyro_x);
        imu_values_.gyro_y = convertGyroToDegreesPerSecond(imu_data_.gyro_y);
        imu_values_.gyro_z = convertGyroToDegreesPerSecond(imu_data_.gyro_z);
        
        imu_values_.accel_x = convertAccelToG(imu_data_.accel_x);
        imu_values_.accel_y = convertAccelToG(imu_data_.accel_y);
        imu_values_.accel_z = convertAccelToG(imu_data_.accel_z);
        
        imu_values_.temperature = convertTempToCelsius_6500(imu_data_.temperature);
    }
    // 检查读取结果
    // if (ret != static_cast<int>(sizeof(buffer))) {
    //     consecutive_read_failures++;
        
    //     // 检查是否超时（超过5秒没有成功读取）
    //     auto now = std::chrono::steady_clock::now();
    //     auto time_since_last_success = std::chrono::duration_cast<std::chrono::seconds>(
    //         now - last_successful_read_time).count();
        
    //     if (time_since_last_success > 5) {
    //         // 报告超时错误
    //         if (global && global->node_) {
    //             interface::msg::HexData error_msg;
    //             error_msg.hex_value = encodeErrorLevel(IMU_TIMEOUT, ErrorEventLevel::OCCUR);
    //             global->sendHexData(error_msg);
    //             RCLCPP_ERROR(logger, "Error event reported: IMU_TIMEOUT (0x%08X) - No data for %ld seconds", 
    //                         IMU_TIMEOUT, time_since_last_success);
    //             current_error_code = IMU_TIMEOUT; // 记录错误状态
    //         }
    //     }
        
    //     // 检查设备是否断连
    //     if (consecutive_read_failures >= MAX_READ_FAILURES) {
    //         if (!device_disconnected_reported && global && global->node_) {
    //             interface::msg::HexData error_msg;
    //             error_msg.hex_value = encodeErrorLevel(IMU_DISCONNECTED, ErrorEventLevel::OCCUR);
    //             global->sendHexData(error_msg);
    //             RCLCPP_ERROR(logger, "Error event reported: IMU_DISCONNECTED (0x%08X) - %d consecutive read failures", 
    //                         IMU_DISCONNECTED, consecutive_read_failures);
    //             current_error_code = IMU_DISCONNECTED; // 记录错误状态
    //             device_disconnected_reported = true;
    //         }
    //     }
        
    //     RCLCPP_WARN(logger, "Failed to read complete IMU data, ret=%d, expected=%zu, failures=%d", 
    //                ret, sizeof(buffer), consecutive_read_failures);
    //     return;
    // }
    
    // 读取成功，重置失败计数
    // consecutive_read_failures = 0;
    // last_successful_read_time = std::chrono::steady_clock::now();
    // device_disconnected_reported = false;
    
    // // 如果之前处于持续性错误状态（NOT_FOUND, DISCONNECTED, TIMEOUT），现在恢复，发布恢复消息
    // if (current_error_code != 0 && current_error_code != IMU_DATA_INVALID) {
    //     if (global && global->node_) {
    //         RCLCPP_INFO(logger, "IMU recovered from error (0x%08X)", current_error_code);
    //         interface::msg::HexData recover_msg;
    //         recover_msg.hex_value = encodeErrorLevel(current_error_code, ErrorEventLevel::RECOVER);
    //         global->sendHexData(recover_msg);
    //     }
    //     current_error_code = 0; // 清除错误状态
    // }

}

// 辅助函数：检查IMU数据有效性
bool ImuComponent::isValidImuData() {
    // 检查NaN和无穷大
    auto isInvalid = [](float val) -> bool {
        return std::isnan(val) || std::isinf(val) || 
               std::abs(val) > 10000.0f;  // 异常大的值
    };
    
    // 检查所有IMU值
    if (isInvalid(imu_values_.gyro_x) || isInvalid(imu_values_.gyro_y) || isInvalid(imu_values_.gyro_z) ||
        isInvalid(imu_values_.accel_x) || isInvalid(imu_values_.accel_y) || isInvalid(imu_values_.accel_z)) {
        return false;
    }
    
    // 检查角速度是否在合理范围内（±2000°/s）
    if (std::abs(imu_values_.gyro_x) > 2000.0f || 
        std::abs(imu_values_.gyro_y) > 2000.0f || 
        std::abs(imu_values_.gyro_z) > 2000.0f) {
        return false;
    }
    
    // 检查加速度是否在合理范围内（±16g）
    if (std::abs(imu_values_.accel_x) > 16.0f || 
        std::abs(imu_values_.accel_y) > 16.0f || 
        std::abs(imu_values_.accel_z) > 16.0f) {
        return false;
    }
    
    return true;
}

void ImuComponent::publishImuData() {
    auto now = this->now();
    
    // 发布 IMU 数据
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = now;
    imu_msg.header.frame_id = "imu_link";
    // 角速度 (rad/s) - 从度/秒转换为弧度/秒，减去零速校准bias
    const double deg2rad = M_PI / 180.0;
    imu_msg.angular_velocity.x = (imu_values_.gyro_x - gyro_x_bias_) * deg2rad;
    imu_msg.angular_velocity.y = (imu_values_.gyro_y - gyro_y_bias_) * deg2rad;
    imu_msg.angular_velocity.z = (imu_values_.gyro_z - gyro_z_bias_) * deg2rad;
    
    // 线性加速度 (m/s²) - 从g转换为m/s²
    imu_msg.linear_acceleration.x = imu_values_.accel_x * 9.81;
    imu_msg.linear_acceleration.y = imu_values_.accel_y * 9.81;
    imu_msg.linear_acceleration.z = imu_values_.accel_z * 9.81;
    
    // 四元数姿态
    imu_msg.orientation.x = orientation_.x();
    imu_msg.orientation.y = orientation_.y();
    imu_msg.orientation.z = orientation_.z();
    imu_msg.orientation.w = orientation_.w();
    
    // 设置协方差矩阵 (假设独立且相等的不确定性)
    double gyro_variance = 0.01;  // 角速度方差
    double accel_variance = 0.1;  // 加速度方差
    double orientation_variance = 0.1;  // 姿态方差
    
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
        temp_msg.temperature = imu_values_.temperature;
        temp_msg.variance = 0.1;  // 温度方差
        
        temp_publisher_->publish(temp_msg);
    }
}

float ImuComponent::convertGyroToDegreesPerSecond(int32_t adc_value) {
    return static_cast<float>(adc_value) / 16.4f;
}

float ImuComponent::convertAccelToG(int32_t adc_value) {
    return static_cast<float>(adc_value) / 2048.0f;
}

float ImuComponent::convertTempToCelsius_6050(int32_t adc_value) {
    return ((static_cast<float>(adc_value) - 25.0f) / 326.8f) + 25.0f;
}
float ImuComponent::convertTempToCelsius_6500(int32_t adc_value) {
    return (static_cast<float>(adc_value) / 333.87f) + 21.0f;
}

float ImuComponent::invSqrt(float x) {
    return 1.0f / std::sqrt(x);
}

void ImuComponent::calculateOrientationFromAccel(double& roll, double& pitch) {
    // 从加速度计计算roll和pitch
    // 注意: 这只在静态或线性加速度较小时准确
    
    // 转换为m/s²
    double ax = imu_values_.accel_x * 9.81;
    double ay = imu_values_.accel_y * 9.81;
    double az = imu_values_.accel_z * 9.81;
    
    // 计算总加速度大小
    double accel_magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    
    // 如果加速度接近重力加速度(9.81 m/s²),则认为设备相对静止,可以使用加速度计估计姿态
    // 如果加速度过大,则可能处于运动状态,加速度计不可靠
    if (accel_magnitude > 7.0 && accel_magnitude < 12.0) {
        // 计算roll (绕X轴旋转)
        roll = std::atan2(ay, az);
        
        // 计算pitch (绕Y轴旋转)
        pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
    }
    // 如果加速度不在合理范围内,保持之前的roll和pitch值
}

void ImuComponent::updateOrientation() {
    auto logger = rclcpp::get_logger("sensor::imu");
    
    // 检查加速度是否为零（无效数据）
    if (imu_values_.accel_x == 0.0f && imu_values_.accel_y == 0.0f && imu_values_.accel_z == 0.0f) {
        return;
    }
    
    // 准备输入数据：角速度（°/s）和加速度（g）
    float gx = imu_values_.gyro_x;
    float gy = imu_values_.gyro_y;
    float gz = imu_values_.gyro_z;
    float ax = imu_values_.accel_x;
    float ay = imu_values_.accel_y;
    float az = imu_values_.accel_z;
    
    // Madgwick 算法变量
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3;
    float q0q0, q1q1, q2q2, q3q3;
    
    // 单位转换：角速度°/s → rad/s
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;
    
    // 归一化加速度
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;
    
    // 预计算常用值
    _2q0 = 2.0f * q0_;
    _2q1 = 2.0f * q1_;
    _2q2 = 2.0f * q2_;
    _2q3 = 2.0f * q3_;
    q0q0 = q0_ * q0_;
    q1q1 = q1_ * q1_;
    q2q2 = q2_ * q2_;
    q3q3 = q3_ * q3_;
    
    // 梯度下降法校正
    s0 = _2q2 * (2.0f * (q1_ * q3_ - q0_ * q2_) - ax) -
         _2q1 * (2.0f * (q0_ * q1_ + q2_ * q3_) - ay);
    s1 = _2q3 * (2.0f * (q1_ * q3_ - q0_ * q2_) - ax) +
         _2q0 * (2.0f * (q0_ * q1_ + q2_ * q3_) - ay) -
         4.0f * q1_ * (1.0f - 2.0f * (q1q1 + q2q2) - az);
    s2 = -_2q0 * (2.0f * (q1_ * q3_ - q0_ * q2_) - ax) +
         _2q3 * (2.0f * (q0_ * q1_ + q2_ * q3_) - ay) -
         4.0f * q2_ * (1.0f - 2.0f * (q1q1 + q2q2) - az);
    s3 = _2q1 * (2.0f * (q1_ * q3_ - q0_ * q2_) - ax) +
         _2q2 * (2.0f * (q0_ * q1_ + q2_ * q3_) - ay);
    
    // 归一化梯度
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;
    
    // 四元数微分方程
    qDot1 = 0.5f * (-q1_ * gx - q2_ * gy - q3_ * gz) - madgwick_beta_ * s0;
    qDot2 = 0.5f * ( q0_ * gx + q2_ * gz - q3_ * gy) - madgwick_beta_ * s1;
    qDot3 = 0.5f * ( q0_ * gy - q1_ * gz + q3_ * gx) - madgwick_beta_ * s2;
    qDot4 = 0.5f * ( q0_ * gz + q1_ * gy - q2_ * gx) - madgwick_beta_ * s3;
    
    // 积分更新
    q0_ += qDot1 / madgwick_sample_freq_;
    q1_ += qDot2 / madgwick_sample_freq_;
    q2_ += qDot3 / madgwick_sample_freq_;
    q3_ += qDot4 / madgwick_sample_freq_;
    
    // 归一化四元数
    recipNorm = invSqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;
    
    // 更新 tf2::Quaternion（用于 ROS2 消息）
    orientation_.setW(q0_);
    orientation_.setX(q1_);
    orientation_.setY(q2_);
    orientation_.setZ(q3_);
    
#ifndef NDEBUG
    // 计算并打印欧拉角（单位：度）
    float roll, pitch, yaw;

    roll  = std::atan2(2.0f * (q0_ * q1_ + q2_ * q3_),
                       1.0f - 2.0f * (q1_ * q1_ + q2_ * q2_));
    pitch = std::asin(2.0f * (q0_ * q2_ - q3_ * q1_));
    yaw   = std::atan2(2.0f * (q0_ * q3_ + q1_ * q2_),
                       1.0f - 2.0f * (q2_ * q2_ + q3_ * q3_));

    const float RAD2DEG = 57.29578f;
    roll  *= RAD2DEG;
    pitch *= RAD2DEG;
    yaw   *= RAD2DEG;

    RCLCPP_DEBUG(logger, "Euler Angles - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°",
                roll, pitch, yaw);
#endif
    
    orientation_initialized_ = true;
}

void ImuComponent::chassisTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    latest_wheel_vx_ = msg->twist.linear.x;
    latest_wheel_vyaw_ = msg->twist.angular.z;
}

bool ImuComponent::detectZeroVelocity() {
    auto now = this->now();

    // 条件1: 线速度和角速度同时低于阈值
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

    // 条件2: 加速度方差低于阈值
    if (accel_mag_history_.size() < kAccelWindowSize) {
        return false;  // 数据不足
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

    // 条件3: 距上次校准间隔大于阈值
    if ((now - last_calibration_time_).seconds() < min_calibration_interval_) {
        return false;
    }

    return true;
}

void ImuComponent::calibrateGyroBias() {
    const double alpha = gyro_bias_ema_alpha_;
    gyro_x_bias_ = alpha * imu_values_.gyro_x + (1.0 - alpha) * gyro_x_bias_;
    gyro_y_bias_ = alpha * imu_values_.gyro_y + (1.0 - alpha) * gyro_y_bias_;
    gyro_z_bias_ = alpha * imu_values_.gyro_z + (1.0 - alpha) * gyro_z_bias_;
    last_calibration_time_ = this->now();
}
