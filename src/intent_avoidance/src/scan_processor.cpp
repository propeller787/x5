#include "intent_avoidance/scan_processor.h"

namespace intent_avoidance {

ScanProcessor::ScanProcessor(const Config& config)
    : config_(config)
{
}

void ScanProcessor::processScan(const sensor_msgs::msg::LaserScan& scan)
{
    ObstacleSectorList sectors;

    if (scan.ranges.empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_sectors_ = sectors;
        return;
    }

    const double angle_min = scan.angle_min;
    const double angle_increment = scan.angle_increment;
    const double range_min = scan.range_min;
    const double range_max = scan.range_max;
    const double threshold = config_.safe_distance_threshold;

    // 逐射线处理，聚合连续阻塞角度为扇区
    ObstacleSector current_sector;
    bool in_sector = false;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        double range = scan.ranges[i];
        double angle_radar = angle_min + i * angle_increment;

        // 跳过无效点
        if (std::isnan(range) || std::isinf(range) || range < range_min || range > range_max) {
            // 视为无数据，中断当前扇区
            if (in_sector) {
                sectors.push_back(current_sector);
                in_sector = false;
            }
            continue;
        }

        // 转换到base_link坐标系
        auto [angle_base, range_base] = beamToBaseLink(angle_radar, range, config_.mount);

        bool blocked = (range_base < threshold);

        if (blocked) {
            if (!in_sector) {
                current_sector.angle_start = angle_base;
                current_sector.min_distance = range_base;
                in_sector = true;
            }
            current_sector.angle_end = angle_base;
            if (range_base < current_sector.min_distance) {
                current_sector.min_distance = range_base;
            }
        } else {
            if (in_sector) {
                // 间隙太小则合并（防止离散噪声导致扇区碎片化）
                double gap = angle_base - current_sector.angle_end;
                if (gap < config_.min_blocked_angle_span) {
                    // 合并：延长当前扇区
                    continue;  // 跳过这个自由点，保持扇区连续
                }
                sectors.push_back(current_sector);
                in_sector = false;
            }
        }
    }

    // 帧尾收尾
    if (in_sector) {
        sectors.push_back(current_sector);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    latest_sectors_ = std::move(sectors);
}

ObstacleSectorList ScanProcessor::getSectors() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return latest_sectors_;
}

std::pair<double, double> ScanProcessor::beamToBaseLink(
    double angle_radar, double range,
    const RadarMount& mount)
{
    // 雷达帧下的点坐标
    double x_r = range * std::cos(angle_radar);
    double y_r = range * std::sin(angle_radar);

    // 刚性变换到base_link（无TF2，纯几何）
    double cos_yaw = std::cos(mount.yaw);
    double sin_yaw = std::sin(mount.yaw);
    double x_v = mount.x + x_r * cos_yaw - y_r * sin_yaw;
    double y_v = mount.y + x_r * sin_yaw + y_r * cos_yaw;

    double angle_base = std::atan2(y_v, x_v);
    double range_base = std::hypot(x_v, y_v);

    return {angle_base, range_base};
}

}  // namespace intent_avoidance
