#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "shared_control_assist/door_detector.hpp"
#include "shared_control_assist/local_perception.hpp"
#include "shared_control_assist/types.hpp"

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace shared_control_assist {

// 单节点共享控制辅助系统。
//
// 外部仍然只有一个 ROS2 Node：
// - 订阅摇杆、车速、三路水平雷达、辅助开关。
// - 发布最终 /nav/control。
// - 发布点云、局部栅格、状态、Marker 等调试信息。
//
// 内部拆分：
// - LocalPerception 负责局部障碍距离和 OccupancyGrid。
// - DoorDetector 负责门洞候选与稳定性判断。
// - 本类负责 ROS IO、主控制循环、窄门状态机、共享控制、安全裁剪。
//
// 设计原则：
// - 不使用 Nav2，不做全局自主导航。
// - 用户摇杆始终是控制意图来源。
// - 软件安全裁剪位于最终输出前。
class SharedControlAssistNode : public rclcpp::Node {
public:
    SharedControlAssistNode();

private:
    // 声明并加载 ROS 参数。默认值来自 AssistParams，YAML 可覆盖。
    void declareAndLoadParams();

    // 创建订阅、发布、TF buffer/listener 和控制定时器。
    void createIo();

    // 摇杆控制输入回调。只缓存最新命令和时间戳，实际计算在 onTimer 中完成。
    void onControl(const geometry_msgs::msg::Twist::SharedPtr msg);

    // 车辆实际速度输入。用于估计门洞状态机中车辆通过距离。
    void onWheelTwist(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    // 辅助开关。false 时进入 MANUAL_ONLY。
    void onAssistEnable(const std_msgs::msg::Bool::SharedPtr msg);

    // 单路 LaserScan 回调。
    //
    // 处理流程：
    // 1. 通过 TF 将雷达点转换到 base_frame。
    // 2. 过滤无效距离、车体 footprint 内部点、过远点。
    // 3. 按 LidarSlot 缓存点云、时间戳、TF 健康状态。
    void onLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr msg, LidarSlot slot);

    // 主控制循环，按 control_rate 定时执行。
    //
    // 高层流程：
    // 1. 取摇杆、车速和雷达点快照。
    // 2. 做局部感知和门洞检测。
    // 3. 更新窄门状态机。
    // 4. 生成普通避障或窄门辅助候选速度。
    // 5. 做安全裁剪和加速度限制。
    // 6. 发布 /nav/control 和调试信息。
    void onTimer();

    // 从三路缓存中取出仍然新鲜的点云。
    // fresh_lidar_count 返回本周期满足 lidar_timeout 的雷达路数。
    std::vector<Point2D> currentPointsLocked(const rclcpp::Time& now, int& fresh_lidar_count) const;

    // 更新窄门状态机。
    //
    // 注意：
    // - SEARCH 阶段只等待候选稳定，不启用强自动转向。
    // - 用户松杆、反向、大角速度取消、安全停车、候选丢失都会进入 ABORT。
    void updateDoorState(
        const geometry_msgs::msg::Twist& user_cmd,
        const DoorCandidate& door,
        const ObstacleDistance& dist,
        const rclcpp::Time& now,
        double dt,
        double forward_velocity);

    // 切换状态并重置该状态内的行驶距离计数。
    void setDoorState(DoorState state, const rclcpp::Time& now);

    // 普通障碍辅助：保留用户线速度/角速度意图，只做最大速度限制。
    geometry_msgs::msg::Twist computeObstacleAssist(const geometry_msgs::msg::Twist& user_cmd) const;

    // 窄门辅助：限制前进速度，并将用户角速度与自动居中角速度混合。
    geometry_msgs::msg::Twist computeDoorAssist(
        const geometry_msgs::msg::Twist& user_cmd,
        const DoorCandidate& door) const;

    // 最终安全裁剪。
    //
    // 输入为候选速度 cmd，输出为允许发送到底盘的安全速度。
    // 这里处理：
    // - 雷达/TF 健康检查。
    // - 前后停车区急停。
    // - 左右侧过近时限制危险方向转向。
    // - 慢速区连续降速。
    geometry_msgs::msg::Twist applySafetyLimit(
        const geometry_msgs::msg::Twist& cmd,
        const ObstacleDistance& dist,
        int fresh_lidar_count,
        const rclcpp::Time& now);

    // 加速度限制。急停时会跳过该限制，直接输出零速度。
    geometry_msgs::msg::Twist applyAccelerationLimit(
        const geometry_msgs::msg::Twist& cmd,
        double dt,
        bool emergency) const;

    // 发布融合后的 base_frame 点云，供 RViz 查看三雷达合并效果。
    void publishPointCloud(const std::vector<Point2D>& points, const rclcpp::Time& stamp);

    // 发布调试信息：栅格、状态 JSON、原因文本、门洞置信度、Marker。
    void publishDebug(
        const std::vector<Point2D>& points,
        const nav_msgs::msg::OccupancyGrid& grid,
        const DoorCandidate& door,
        const ObstacleDistance& dist,
        const geometry_msgs::msg::Twist& user_cmd,
        const geometry_msgs::msg::Twist& output_cmd,
        const rclcpp::Time& now);

    // 摇杆命令是否超时。
    bool isCommandTimedOut(const rclcpp::Time& now) const;

    // 枚举转字符串，主要用于 /assist/state 调试输出。
    std::string modeName() const;
    std::string doorStateName() const;

    // 全局参数。LocalPerception 和 DoorDetector 以引用方式持有该对象。
    AssistParams params_;

    // 内部算法模块。它们不直接创建 ROS topic，便于后续单元测试。
    LocalPerception perception_;
    DoorDetector door_detector_;

    // ---------------- ROS 订阅 ----------------
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr control_sub_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr assist_enable_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr left_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr right_scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr back_scan_sub_;

    // ---------------- ROS 发布 ----------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr control_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reason_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr door_score_pub_;

    // 控制定时器和 TF 查询对象。
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 保护以下缓存数据。当前 rclcpp 默认单线程 spin 时竞争较少，
    // 但保留 mutex 便于未来切到 MultiThreadedExecutor。
    mutable std::mutex mutex_;

    // 最近一次摇杆命令、车速和输出命令。
    geometry_msgs::msg::Twist latest_user_cmd_;
    geometry_msgs::msg::Twist latest_wheel_twist_;
    geometry_msgs::msg::Twist last_output_cmd_;
    bool has_user_cmd_ = false;
    bool has_wheel_twist_ = false;
    bool assist_enabled_ = true;

    // 三路雷达点缓存。索引顺序由 LidarSlot 定义。
    std::array<std::vector<Point2D>, 3> scan_points_;
    std::array<rclcpp::Time, 3> scan_stamps_{
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Time(0, 0, RCL_ROS_TIME)};
    std::array<bool, 3> scan_valid_{false, false, false};

    // 各类时间戳。用于超时、状态保持和调试节流。
    rclcpp::Time last_user_cmd_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_timer_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_output_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_debug_publish_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time state_enter_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time emergency_until_{0, 0, RCL_ROS_TIME};

    // 每路雷达 TF 失败起始时间。任意一路持续失败超过 tf_fail_timeout，
    // 都会触发安全保护；正常收到该路 TF 后清零。
    std::array<rclcpp::Time, 3> tf_fail_since_{
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Time(0, 0, RCL_ROS_TIME),
        rclcpp::Time(0, 0, RCL_ROS_TIME)};

    // 当前辅助模式、窄门状态和状态内累计前进距离。
    AssistMode mode_ = AssistMode::OBSTACLE_ASSIST;
    DoorState door_state_ = DoorState::IDLE;
    double travel_in_state_ = 0.0;

    // 最近一次调试原因，发布到 /assist/debug/reason 和 /assist/state。
    std::string debug_reason_ = "starting";
};

}  // namespace shared_control_assist
