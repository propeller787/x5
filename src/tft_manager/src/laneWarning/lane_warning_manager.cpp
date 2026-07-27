#include "laneWarning/lane_warning_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <utils/topic.h>

namespace tft_service {

LaneWarningManager& LaneWarningManager::instance() {
    static LaneWarningManager inst;
    return inst;
}

void LaneWarningManager::init(rclcpp::Node* node) {
    if (!node) return;
    
    std::lock_guard<std::mutex> lock(mutex_);
    if (node_) return;
    
    node_ = node;
    auto logger = rclcpp::get_logger("tft_service::laneWarning::lane_warning_manager");
    
    // 输入参数 - 倒车轨迹配置（根据视觉参考图）
    node_->declare_parameter("lane_warning.left_start_x", 119.0);
    node_->declare_parameter("lane_warning.left_start_y", 779.0);
    node_->declare_parameter("lane_warning.right_start_x", 600.0);
    node_->declare_parameter("lane_warning.right_start_y", 779.0);
    node_->declare_parameter("lane_warning.left_end_x", 265.0);
    node_->declare_parameter("lane_warning.left_end_y", 495.0);
    node_->declare_parameter("lane_warning.right_end_x", 435.0);
    node_->declare_parameter("lane_warning.right_end_y", 495.0);
    node_->declare_parameter("lane_warning.max_curvature", 0.02);
    
    left_start_x_ = node_->get_parameter("lane_warning.left_start_x").as_double();
    left_start_y_ = node_->get_parameter("lane_warning.left_start_y").as_double();
    right_start_x_ = node_->get_parameter("lane_warning.right_start_x").as_double();
    right_start_y_ = node_->get_parameter("lane_warning.right_start_y").as_double();
    left_end_x_ = node_->get_parameter("lane_warning.left_end_x").as_double();
    left_end_y_ = node_->get_parameter("lane_warning.left_end_y").as_double();
    right_end_x_ = node_->get_parameter("lane_warning.right_end_x").as_double();
    right_end_y_ = node_->get_parameter("lane_warning.right_end_y").as_double();
    max_curvature_ = node_->get_parameter("lane_warning.max_curvature").as_double();
    
    RCLCPP_INFO(logger, "LaneWarningManager initialized (Visual Trajectory):");
    RCLCPP_INFO(logger, "  Left Start: (%.2f, %.2f)", left_start_x_, left_start_y_);
    RCLCPP_INFO(logger, "  Right Start: (%.2f, %.2f)", right_start_x_, right_start_y_);
    RCLCPP_INFO(logger, "  Left End: (%.2f, %.2f)", left_end_x_, left_end_y_);
    RCLCPP_INFO(logger, "  Right End: (%.2f, %.2f)", right_end_x_, right_end_y_);
    RCLCPP_INFO(logger, "  Max Curvature: %.4f", max_curvature_);
    
    auto qos = rclcpp::QoS(10);
    joystick_sub_ = node_->create_subscription<interface::msg::Joystick>(
        joystickTopic, qos,
        [this](const interface::msg::Joystick::SharedPtr msg) { joystickCallback(msg); });
    
    warning_trajectory_pub_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>(
        laneWarningPointTopic, 10);
}

void LaneWarningManager::joystickCallback(const interface::msg::Joystick::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    int16_t rocker_angle = msg->rocker_angle;
    std::vector<float> output_data;
    calculateTrajectory(output_data, rocker_angle);
    publishTrajectory(output_data);
}

std::vector<float> LaneWarningManager::calculateBezierCurve(
    float start_x, float start_y, 
    float ctrl_x, float ctrl_y, 
    float end_x, float end_y, 
    int num_points) {
    
    std::vector<float> points;
    if (num_points < 2) {
        return points;
    }
    
    for (int i = 0; i < num_points; ++i) {
        float t = static_cast<float>(i) / (num_points - 1);
        float t2 = t * t;
        float mt = 1.0f - t;
        float mt2 = mt * mt;
        
        // 二阶贝塞尔曲线
        float x = mt2 * start_x + 2.0f * mt * t * ctrl_x + t2 * end_x;
        float y = mt2 * start_y + 2.0f * mt * t * ctrl_y + t2 * end_y;
        
        points.push_back(x);
        points.push_back(y);
    }
    
    return points;
}

void LaneWarningManager::calculateTrajectory(std::vector<float>& output_data, int16_t rocker_angle) {
    // rocker_angle 180 / -180度为直线倒行 [-90,-180] 为右倒行 [90,180] 为左倒行
    float angle_deg = static_cast<float>(rocker_angle);
    int angle_deg_int = 90.0f;
    if (angle_deg <= -90.0f) {
        // 左倒行：[-180, -90] → [0, 45]
        angle_deg = ((angle_deg + 180.0f) / 90.0f) * angle_deg_int;
    } else if (angle_deg >= 90.0f) {
        // 右倒行：[90, 180] → [-45, 0]
        angle_deg = ((angle_deg - 180.0f) / 90.0f) * angle_deg_int;
    } else {
        return;
    }    
    float left_start_x = static_cast<float>(left_start_x_);
    float left_start_y = static_cast<float>(left_start_y_);
    float right_start_x = static_cast<float>(right_start_x_);
    float right_start_y = static_cast<float>(right_start_y_);
    // 直走时的终点（默认汇聚点）
    float straight_left_end_x = static_cast<float>(left_end_x_);
    float straight_left_end_y = static_cast<float>(left_end_y_);
    float straight_right_end_x = static_cast<float>(right_end_x_);
    float straight_right_end_y = static_cast<float>(right_end_y_);
    
    // 计算水平偏移量（转向角越大，偏移越大）
    float end_width = straight_right_end_x - straight_left_end_x;
    
    // 增大最大偏移量，增强弯曲效果
    float max_offset = end_width * static_cast<float>(max_curvature_) * 35.0f;
    
    // 根据转向角计算偏移（使用平方关系，使弯曲更明显）
    float normalized_angle = angle_deg / angle_deg_int;  // 归一化到 [-1, 1]
    float offset = max_offset * normalized_angle;
    
    // 计算实际终点
    float left_end_x = straight_left_end_x - offset;
    float left_end_y = straight_left_end_y;
    float right_end_x = straight_right_end_x - offset;
    float right_end_y = straight_right_end_y;
    
    // 优化贝塞尔曲线控制点，增强弯曲效果
    // 控制点偏移方向与终点偏移方向相反，形成自然的曲线
    float ctrl_offset = offset * 1.5f;  // 增大控制点偏移比例
    
    // 使用不对称控制点位置，让曲线更自然
    float t = 0.6f;  // 控制点偏向终点方向
    // 左侧轨迹控制点
    float left_ctrl_x = left_start_x * (1.0f - t) + left_end_x * t + ctrl_offset;
    float left_ctrl_y = left_start_y * (1.0f - t) + left_end_y * t;
    
    // 右侧轨迹控制点
    float right_ctrl_x = right_start_x * (1.0f - t) + right_end_x * t + ctrl_offset;
    float right_ctrl_y = right_start_y * (1.0f - t) + right_end_y * t;
    
    // 生成6个点的轨迹曲线
    left_trajectory_ = calculateBezierCurve(
        left_start_x, left_start_y,
        left_ctrl_x, left_ctrl_y,
        left_end_x, left_end_y,
        6);
    
    right_trajectory_ = calculateBezierCurve(
        right_start_x, right_start_y,
        right_ctrl_x, right_ctrl_y,
        right_end_x, right_end_y,
        6);

    output_data.clear();  // 清空向量 一定清空防止内部缓存其他数据
    
    // 左侧轨迹（6个点，12个坐标）
    if (left_trajectory_.size() >= 12) {
        output_data.insert(output_data.end(), left_trajectory_.begin(), left_trajectory_.begin() + 12);
    } else {
        output_data.insert(output_data.end(), 12, 0.0f);
    }
    
    // 右侧轨迹（6个点，12个坐标）
    if (right_trajectory_.size() >= 12) {
        output_data.insert(output_data.end(), right_trajectory_.begin(), right_trajectory_.begin() + 12);
    } else {
        output_data.insert(output_data.end(), 12, 0.0f);
    }
}

void LaneWarningManager::publishTrajectory(const std::vector<float>& output_data) {
    if (!warning_trajectory_pub_) return;
    
    std_msgs::msg::Float32MultiArray msg;
    msg.data = output_data;
    warning_trajectory_pub_->publish(msg);
}

} // namespace tft_service