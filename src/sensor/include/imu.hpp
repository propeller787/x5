#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <utils/topic.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <deque>
#include <string>
#include <memory>
#include <chrono>

namespace sensor::imu {

class ImuComponent : public rclcpp::Node {
public:
    ImuComponent(const rclcpp::NodeOptions& options);
    ~ImuComponent();

private:
    bool setupDevice();
    void readImuData();
    void publishImuData();
    void onTimer();
    
    // MPU6050 数据转换函数
    float convertGyroToDegreesPerSecond(int32_t adc_value);
    float convertAccelToG(int32_t adc_value);
    float convertTempToCelsius_6050(int32_t adc_value);
    float convertTempToCelsius_6500(int32_t adc_value);
    
    // 姿态融合函数
    void updateOrientation();
    void calculateOrientationFromAccel(double& roll, double& pitch);

    // 零速检测与陀螺仪bias校准
    void chassisTwistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    bool detectZeroVelocity();
    void calibrateGyroBias();
    
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

    struct ImuValues_JY61P{
        float gyro_x, gyro_y, gyro_z;  // 角速度 (°/S)
        float accel_x, accel_y, accel_z; // 加速度 (g)
        float roll, pitch, yaw;          // 欧拉角 (°)
        float temperature;              // 温度 (°C)
        float q0, q1, q2, q3;           // 四元数 (w, x, y, z)
    } imu_values_jy61p_;
      
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

    // 底盘速度反馈订阅（用于零速检测）
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr chassis_twist_sub_;
    double latest_wheel_vx_ = 0.0;
    double latest_wheel_vyaw_ = 0.0;
    std::string chassis_twist_topic_;

    // 零速检测参数
    double zero_vel_linear_threshold_;   // 线速度阈值 (m/s)
    double zero_vel_angular_threshold_;  // 角速度阈值 (rad/s)
    double zero_vel_duration_;           // 零速持续时间阈值 (s)
    double accel_variance_threshold_;   // 加速度幅值方差阈值 (g²)
    double min_calibration_interval_;   // 最小校准间隔 (s)
    bool enable_zero_vel_calib_;        // 零速校准开关
    double gyro_bias_ema_alpha_;        // bias估计EMA平滑系数

    // 零速检测状态
    rclcpp::Time stationary_start_time_;
    bool stationary_candidate_ = false;
    bool in_zero_vel_ = false;
    rclcpp::Time last_calibration_time_;

    // 加速度幅值历史（方差计算用）
    static constexpr size_t kAccelWindowSize = 20;
    std::deque<double> accel_mag_history_;

    // 陀螺仪bias估计 (deg/s)
    double gyro_x_bias_ = 0.0;
    double gyro_y_bias_ = 0.0;
    double gyro_z_bias_ = 0.0;
};

} // namespace sensor::imu
