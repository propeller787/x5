/**
 * @file tracking.cpp
 * @author likang (likang@freego.com)
 * @brief 跟踪实现
 * @version 0.1
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */

#include "tracking.h"
#include <iostream>
#include <algorithm>
#include <cmath>

Tracking::Tracking() : next_object_id_(1), wheel_speed_left_(0.0), wheel_speed_right_(0.0) {
    std::cout << "初始化跟踪模块" << std::endl;
}

Tracking::~Tracking() {
    std::cout << "跟踪模块析构" << std::endl;
}

void Tracking::setTrackingCallback(std::function<void(const std::vector<TrackedObject>&)> callback) {
    tracking_callback_ = callback;
}

void Tracking::updateTracking(const std::vector<ClusterInfo>& clusters, const VehicleState& vehicle_state) {
    std::cout << "更新跟踪，输入聚类数: " << clusters.size() << std::endl;
    
}

std::vector<TrackedObject> Tracking::predictObjectPositions(double time_delta) {
    auto predicted_objects = tracked_objects_;
    
    for (auto& obj : predicted_objects) {
        // 基于目标自身运动预测
        obj.predicted_x = obj.last_cluster.center_x + obj.velocity_x * time_delta;
        obj.predicted_y = obj.last_cluster.center_y + obj.velocity_y * time_delta;
        
        std::cout << "目标 " << obj.id << " 预测位置: (" << obj.predicted_x 
                  << ", " << obj.predicted_y << ")" << std::endl;
    }
    
    return predicted_objects;
}

std::vector<std::pair<int, int>> Tracking::associateClustersWithTracks(const std::vector<ClusterInfo>& clusters) {
    std::vector<std::pair<int, int>> associations; // (cluster_index, track_index)
    std::vector<bool> cluster_used(clusters.size(), false);
    std::vector<bool> track_used(tracked_objects_.size(), false);
    
    // 计算所有距离并排序
    std::vector<std::tuple<double, int, int>> distances; // (distance, cluster_index, track_index)
    
    for (size_t i = 0; i < clusters.size(); ++i) {
        for (size_t j = 0; j < tracked_objects_.size(); ++j) {
            double distance = calculateDistance(clusters[i], tracked_objects_[j]);
            if (distance <= tracking_params_.association_threshold) {
                distances.emplace_back(distance, i, j);
            }
        }
    }
    
    // 按距离排序
    std::sort(distances.begin(), distances.end());
    
    // 贪心分配
    for (const auto& [distance, cluster_idx, track_idx] : distances) {
        if (!cluster_used[cluster_idx] && !track_used[track_idx]) {
            associations.emplace_back(cluster_idx, track_idx);
            cluster_used[cluster_idx] = true;
            track_used[track_idx] = true;
        }
    }
    
    std::cout << "数据关联完成，关联对数: " << associations.size() << std::endl;
    return associations;
}

void Tracking::updateExistingTracks(const std::vector<ClusterInfo>& clusters, 
                                   const std::vector<std::pair<int, int>>& associations) {
    for (const auto& [cluster_idx, track_idx] : associations) {
        auto& track = tracked_objects_[track_idx];
        const auto& cluster = clusters[cluster_idx];
        
        // 计算速度
        double dt = (track.last_update_time.seconds() > 0) ? 
                    (rclcpp::Clock().now().seconds() - track.last_update_time.seconds()) : 0.1;
        
        if (dt > 0) {
            track.velocity_x = (cluster.center_x - track.last_cluster.center_x) / dt;
            track.velocity_y = (cluster.center_y - track.last_cluster.center_y) / dt;
        }
        
        // 更新跟踪信息
        track.last_cluster = cluster;
        track.predicted_x = cluster.center_x;
        track.predicted_y = cluster.center_y;
        track.width = cluster.width;
        track.height = cluster.height;
        track.age++;
        track.missed_count = 0;
        track.last_update_time = rclcpp::Clock().now();
        
        std::cout << "更新跟踪目标 " << track.id << ", 速度: (" << track.velocity_x 
                  << ", " << track.velocity_y << ")" << std::endl;
    }
}

void Tracking::createNewTracks(const std::vector<ClusterInfo>& clusters, 
                              const std::vector<std::pair<int, int>>& associations) {
    std::vector<bool> cluster_used(clusters.size(), false);
    
    // 标记已关联的聚类
    for (const auto& [cluster_idx, track_idx] : associations) {
        cluster_used[cluster_idx] = true;
    }
    
    // 为未关联的聚类创建新跟踪目标
    for (size_t i = 0; i < clusters.size(); ++i) {
        if (!cluster_used[i]) {
            TrackedObject new_track;
            new_track.id = next_object_id_++;
            new_track.last_cluster = clusters[i];
            new_track.predicted_x = clusters[i].center_x;
            new_track.predicted_y = clusters[i].center_y;
            new_track.width = clusters[i].width;
            new_track.height = clusters[i].height;
            new_track.velocity_x = 0.0;
            new_track.velocity_y = 0.0;
            new_track.age = 1;
            new_track.missed_count = 0;
            new_track.last_update_time = rclcpp::Clock().now();
            
            tracked_objects_.push_back(new_track);
            
            std::cout << "创建新跟踪目标 " << new_track.id << " 在位置 (" 
                      << new_track.predicted_x << ", " << new_track.predicted_y << ")" << std::endl;
        }
    }
}

void Tracking::removeLostTracks() {
    // auto it = std::remove_if(tracked_objects_.begin(), tracked_objects_.end(),
    //     [this](const TrackedObject& track) {
    //         track.missed_count++;
    //         return track.missed_count > tracking_params_.max_missed_frames;
    //     });
    
    // if (it != tracked_objects_.end()) {
    //     std::cout << "删除 " << std::distance(it, tracked_objects_.end()) << " 个丢失的跟踪目标" << std::endl;
    //     tracked_objects_.erase(it, tracked_objects_.end());
    // }
}

double Tracking::calculateDistance(const ClusterInfo& cluster, const TrackedObject& track) {
    double dx = cluster.center_x - track.predicted_x;
    double dy = cluster.center_y - track.predicted_y;
    return std::sqrt(dx * dx + dy * dy);
}

void Tracking::predictTrackPosition(TrackedObject& track, const VehicleState& vehicle_state, double time_delta) {
    // 这里可以实现更复杂的预测算法
    // 例如考虑车辆运动对目标观测的影响
    track.predicted_x = track.last_cluster.center_x + track.velocity_x * time_delta;
    track.predicted_y = track.last_cluster.center_y + track.velocity_y * time_delta;
}

std::vector<TrackedObject> Tracking::predictObjectPositionsBasedOnVehicleMotion(const VehicleState& vehicle_state, double time_delta) {
    auto predicted_objects = tracked_objects_;
    
    for (auto& obj : predicted_objects) {
        // 目标静止假设：所有雷达目标都理解为静止目标
        // 只通过车辆自身反馈速度进行目标预测
        
        // 车辆运动对目标相对位置的影响
        double vehicle_dx = -vehicle_state.linear_velocity * time_delta;  // 车辆前进，目标相对后退
        double vehicle_dtheta = -vehicle_state.angular_velocity * time_delta;  // 车辆左转，目标相对右转
        
        // 目标在车辆坐标系中的新位置
        double cos_theta = std::cos(vehicle_dtheta);
        double sin_theta = std::sin(vehicle_dtheta);
        
        double new_x = obj.last_cluster.center_x * cos_theta - obj.last_cluster.center_y * sin_theta + vehicle_dx;
        double new_y = obj.last_cluster.center_x * sin_theta + obj.last_cluster.center_y * cos_theta;
        
        obj.predicted_x = new_x;
        obj.predicted_y = new_y;
        
        std::cout << "目标 " << obj.id << " 基于车辆运动预测位置: (" << obj.predicted_x 
                  << ", " << obj.predicted_y << "), 车辆速度: (" << vehicle_state.linear_velocity 
                  << ", " << vehicle_state.angular_velocity << "), 时间差: " << time_delta << "s" << std::endl;
    }
    
    return predicted_objects;
}

void Tracking::updateWheelSpeed(double left_wheel_speed, double right_wheel_speed) {
    wheel_speed_left_ = left_wheel_speed;
    wheel_speed_right_ = right_wheel_speed;
    // std::cout << "Tracking: 更新轮速数据 - 左轮=" << left_wheel_speed 
    //           << " m/s, 右轮=" << right_wheel_speed << " m/s" << std::endl;
}
