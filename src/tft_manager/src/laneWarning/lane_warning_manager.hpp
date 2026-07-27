#pragma once

#include <rclcpp/rclcpp.hpp>
#include <interface/msg/joystick.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <mutex>

namespace tft_service {
class LaneWarningManager {
public:
    static LaneWarningManager& instance();

    void init(rclcpp::Node* node);

private:
    LaneWarningManager() = default;
    ~LaneWarningManager() = default;
    LaneWarningManager(const LaneWarningManager&) = delete;
    LaneWarningManager& operator=(const LaneWarningManager&) = delete;

    void joystickCallback(const interface::msg::Joystick::SharedPtr msg);
    void calculateTrajectory(std::vector<float>& output_data, int16_t rocker_angle);
    void publishTrajectory(const std::vector<float>& output_data);

    std::vector<float> calculateBezierCurve(
        float start_x, float start_y, 
        float ctrl_x, float ctrl_y, 
        float end_x, float end_y, 
        int num_points);

private:
    mutable std::mutex mutex_;
    rclcpp::Node* node_{nullptr};
    rclcpp::Subscription<interface::msg::Joystick>::SharedPtr joystick_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr warning_trajectory_pub_;
    
    // 输入参数 - 倒车轨迹配置（根据视觉参考图）
    double left_start_x_{119.0};    // 左起点X坐标（像素）
    double left_start_y_{779.0};    // 左起点Y坐标（像素）
    double right_start_x_{600.0};   // 右起点X坐标（像素）
    double right_start_y_{779.0};   // 右起点Y坐标（像素）
    double left_end_x_{265.0};      // 左终点X坐标（像素，直走时）
    double left_end_y_{495.0};      // 左终点Y坐标（像素）
    double right_end_x_{435.0};     // 右终点X坐标（像素，直走时）
    double right_end_y_{495.0};     // 右终点Y坐标（像素）
    double max_curvature_{0.02};    // 最大曲率系数
    
    // 轨迹输出
    std::vector<float> left_trajectory_;   // 左侧轨迹曲线点阵（x1,y1,x2,y2,...）
    std::vector<float> right_trajectory_;  // 右侧轨迹曲线点阵
    
    // std::vector<float> output_data_;        // 最终输出数据
};

} // namespace tft_service