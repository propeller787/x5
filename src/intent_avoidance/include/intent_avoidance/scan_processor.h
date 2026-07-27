#pragma once

#include "intent_avoidance/types.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cmath>
#include <mutex>

namespace intent_avoidance {

// LaserScan → 阻塞扇区
// 不做TF2，使用标定安装参数做刚性几何变换
class ScanProcessor {
public:
    struct Config {
        double safe_distance_threshold = 1.2;    // 小于此距离判定为阻塞(m)
        double min_blocked_angle_span = 0.017;   // 最小阻塞角度间隙(rad)，≈1°
        RadarMount mount;                         // 雷达安装参数
    };

    explicit ScanProcessor(const Config& config);

    // 接收一帧雷达扫描，转换为base_link坐标系下的阻塞扇区列表
    void processScan(const sensor_msgs::msg::LaserScan& scan);

    // 获取最新阻塞扇区（线程安全）
    ObstacleSectorList getSectors() const;

    // 单条射线从雷达帧转换到车辆坐标系
    // 返回 base_link 下的 (angle_rad, range_m)
    // public: 允许测试和外部使用
    static std::pair<double, double> beamToBaseLink(
        double angle_radar, double range,
        const RadarMount& mount);

private:
    Config config_;
    mutable std::mutex mutex_;
    ObstacleSectorList latest_sectors_;
};

}  // namespace intent_avoidance
