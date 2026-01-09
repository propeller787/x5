#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <utils/topic.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <string>
#include <memory>
#include <chrono>

namespace sensor::imu {

class ImuComponent : public rclcpp::Node {
public:
    ImuComponent(const rclcpp::NodeOptions& options);
    ~ImuComponent();

private:
    void setupDevice();
    void readImuData();
    void publishImuData();
    void onTimer();
    
    // MPU6050 数据转换函数
    float convertGyroToDegreesPerSecond(int32_t adc_value);
    float convertAccelToG(int32_t adc_value);
    float convertTempToCelsius(int32_t adc_value);
    
    // 姿态融合函数
    void updateOrientation();
    void calculateOrientationFromAccel(double& roll, double& pitch);
    
    // 数据有效性检查
    bool isValidImuData();

private:
    // 设备文件描述符
    int device_fd_;
    std::string device_path_;
    
    // ROS2 发布器
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_publisher_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 数据缓冲区 - 与设备驱动格式匹配
    struct ImuData {
        int32_t gyro_x, gyro_y, gyro_z;    // 改为int32_t匹配设备驱动
        int32_t accel_x, accel_y, accel_z;  // 改为int32_t匹配设备驱动
        int32_t temperature;                // 改为int32_t匹配设备驱动
    } imu_data_;
    
    // 转换后的实际值
    struct ImuValues {
        float gyro_x, gyro_y, gyro_z;  // 角速度 (°/S)
        float accel_x, accel_y, accel_z; // 加速度 (g)
        float temperature;              // 温度 (°C)
    } imu_values_;
    
    // 配置参数
    double publish_rate_;  // 发布频率 (Hz)
    bool enable_temperature_ = false; // 是否发布温度数据
    
    // 姿态融合相关 - 使用 Madgwick 算法
    tf2::Quaternion orientation_;  // 当前四元数姿态
    std::chrono::steady_clock::time_point last_update_time_;  // 上次更新时间
    bool orientation_initialized_;  // 是否已初始化姿态
    
    // Madgwick AHRS 算法参数
    float madgwick_beta_;        // 融合参数
    float madgwick_sample_freq_; // 采样频率Hz
    float q0_, q1_, q2_, q3_;    // 四元数 (w, x, y, z)
    
    // Madgwick 算法辅助函数
    static float invSqrt(float x);
};

} // namespace sensor::imu
