/**
 * @file obstacle_predictor.cpp
 * @author likang (likang@freego.com)
 * @brief 障碍物位置预测器 - 基于轮速反馈的实时运动补偿实现
 * @version 0.1
 * @date 2026-04-06
 * @copyright Copyright (c) 2026 Freego Technology Co., Ltd.
 */

#include "obstacle_predictor.h"
#include <cmath>
#include <algorithm>

ObstaclePredictor::ObstaclePredictor(rclcpp::Logger logger)
    : logger_(logger)
{
}

void ObstaclePredictor::updateParams(const ObstaclePredictorParams& params)
{
    std::lock_guard<std::mutex> lock(mutex_);
    params_ = params;
    RCLCPP_INFO(logger_, "[ObstaclePredictor] 参数更新: enable=%d, max_age=%.3f s, drift_ratio=%.2f",
                params_.enable, params_.max_prediction_age, params_.max_distance_drift_ratio);
}

void ObstaclePredictor::updateRadarClusters(const std::vector<ClusterInfo>& clusters,
                                             const std::string& radar_name,
                                             const rclcpp::Time& timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto& cache = radar_caches_[radar_name];
    cache.clusters = clusters;
    cache.original_centers.resize(clusters.size());
    for (size_t i = 0; i < clusters.size(); ++i) {
        cache.original_centers[i] = {clusters[i].center_x, clusters[i].center_y};
    }
    cache.radar_timestamp = timestamp;
    cache.accumulated_dt = 0.0;

    RCLCPP_DEBUG(logger_, "[ObstaclePredictor] 雷达 %s 更新: %zu 个聚类",
                 radar_name.c_str(), clusters.size());
}

void ObstaclePredictor::onWheelSpeedUpdate(double linear_velocity, double angular_velocity,
                                            const rclcpp::Time& timestamp)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!params_.enable) {
        return;
    }

    // 计算距上次轮速更新的时间增量
    double dt = 0.0;
    if (has_wheel_speed_) {
        dt = (timestamp - last_wheel_speed_time_).seconds();
        // 防护异常时间跳变
        if (dt <= 0.0 || dt > 0.2) {
            last_wheel_speed_time_ = timestamp;
            return;
        }
    } else {
        has_wheel_speed_ = true;
        last_wheel_speed_time_ = timestamp;
        return;
    }
    last_wheel_speed_time_ = timestamp;

    // 对每个雷达缓存做增量补偿
    for (auto it = radar_caches_.begin(); it != radar_caches_.end(); ) {
        auto& cache = it->second;

        cache.accumulated_dt += dt;

        // 累计补偿时间超限，丢弃该雷达缓存
        if (cache.accumulated_dt > params_.max_prediction_age) {
            RCLCPP_DEBUG(logger_, "[ObstaclePredictor] 雷达 %s 累计补偿 %.3f s 超限，清除缓存",
                         it->first.c_str(), cache.accumulated_dt);
            cache.clusters.clear();
            ++it;
            continue;
        }

        // 增量补偿
        if (!cache.clusters.empty()) {
            compensateClusters(cache.clusters, cache.original_centers, dt, linear_velocity, angular_velocity);
        }
        ++it;
    }
}

std::vector<ClusterInfo> ObstaclePredictor::getPredictedClusters(const std::string& radar_name) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!params_.enable) {
        // 关闭时返回未补偿的聚类（缓存中仍为最新雷达帧数据，因 enable=false 时不做补偿）
        auto it = radar_caches_.find(radar_name);
        if (it != radar_caches_.end()) {
            return it->second.clusters;
        }
        return {};
    }

    auto it = radar_caches_.find(radar_name);
    if (it == radar_caches_.end()) {
        return {};
    }
    return it->second.clusters;
}

std::map<std::string, std::vector<ClusterInfo>> ObstaclePredictor::getAllPredictedClusters() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::map<std::string, std::vector<ClusterInfo>> result;
    for (const auto& pair : radar_caches_) {
        result[pair.first] = pair.second.clusters;
    }
    return result;
}

double ObstaclePredictor::getRadarDataAge(const std::string& radar_name, const rclcpp::Time& now) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = radar_caches_.find(radar_name);
    if (it == radar_caches_.end()) {
        return 1e6;
    }
    return (now - it->second.radar_timestamp).seconds();
}

// ==================== 私有方法 ====================

void ObstaclePredictor::compensateClusters(std::vector<ClusterInfo>& clusters,
                                            std::vector<OriginalCenter>& original_centers,
                                            double dt, double linear_vel, double angular_vel) const
{
    // 车辆运动学：目标静止，车辆运动 → 目标在车体系中反向移动
    const double vehicle_dtheta = -angular_vel * dt;
    const double vehicle_dx = -linear_vel * dt;
    const double cos_theta = std::cos(vehicle_dtheta);
    const double sin_theta = std::sin(vehicle_dtheta);

    // 就地补偿，同步删除 clusters 和 original_centers 中的无效项
    size_t write_idx = 0;
    for (size_t i = 0; i < clusters.size(); ++i) {
        const double orig_x = (i < original_centers.size()) ? original_centers[i].x : clusters[i].center_x;
        const double orig_y = (i < original_centers.size()) ? original_centers[i].y : clusters[i].center_y;
        if (compensateOneCluster(clusters[i], cos_theta, sin_theta, vehicle_dx, orig_x, orig_y)) {
            if (write_idx != i) {
                clusters[write_idx] = std::move(clusters[i]);
                if (write_idx < original_centers.size() && i < original_centers.size()) {
                    original_centers[write_idx] = original_centers[i];
                }
            }
            ++write_idx;
        }
    }
    clusters.resize(write_idx);
    if (original_centers.size() > write_idx) {
        original_centers.resize(write_idx);
    }
}

bool ObstaclePredictor::compensateOneCluster(ClusterInfo& cluster,
                                              double cos_theta, double sin_theta,
                                              double vehicle_dx,
                                              double orig_x, double orig_y) const
{
    const double old_x = cluster.center_x;
    const double old_y = cluster.center_y;

    // 刚体旋转 + 平移
    const double new_x = old_x * cos_theta - old_y * sin_theta + vehicle_dx;
    const double new_y = old_x * sin_theta + old_y * cos_theta;

    // 校验：目标移到车后方过远
    if (new_x < params_.min_valid_x) {
        return false;
    }

    // 校验：累计距离偏差过大（与原始雷达帧位置比较，而非单步比较）
    const double orig_dist = std::sqrt(orig_x * orig_x + orig_y * orig_y);
    const double new_dist = std::sqrt(new_x * new_x + new_y * new_y);
    if (std::abs(new_dist - orig_dist) > params_.max_distance_drift_ratio * orig_dist + 1.0) {
        return false;
    }

    // 更新中心位置
    const double dx_offset = new_x - old_x;
    const double dy_offset = new_y - old_y;
    cluster.center_x = new_x;
    cluster.center_y = new_y;

    // 更新点云
    if (cluster.points && !cluster.points->points.empty()) {
        for (auto& point : cluster.points->points) {
            const double px = point.x;
            const double py = point.y;
            point.x = px * cos_theta - py * sin_theta + vehicle_dx;
            point.y = px * sin_theta + py * cos_theta;
        }
        // 有旋转时不能仅用中心平移更新 AABB，否则与点云不一致；由点云重建包围盒
        double mn_x = cluster.points->points[0].x;
        double mx_x = mn_x;
        double mn_y = cluster.points->points[0].y;
        double mx_y = mn_y;
        double mn_z = cluster.points->points[0].z;
        double mx_z = mn_z;
        for (const auto& point : cluster.points->points) {
            mn_x = std::min(mn_x, static_cast<double>(point.x));
            mx_x = std::max(mx_x, static_cast<double>(point.x));
            mn_y = std::min(mn_y, static_cast<double>(point.y));
            mx_y = std::max(mx_y, static_cast<double>(point.y));
            mn_z = std::min(mn_z, static_cast<double>(point.z));
            mx_z = std::max(mx_z, static_cast<double>(point.z));
        }
        cluster.min_x = mn_x;
        cluster.max_x = mx_x;
        cluster.min_y = mn_y;
        cluster.max_y = mx_y;
        cluster.min_z = mn_z;
        cluster.max_z = mx_z;
    } else {
        // 无点云时只能平移 AABB（存在角速度时仍不严格，与决策主要用 center 一致）
        cluster.min_x += dx_offset;
        cluster.max_x += dx_offset;
        cluster.min_y += dy_offset;
        cluster.max_y += dy_offset;
    }

    return true;
}
