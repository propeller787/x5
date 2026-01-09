/**
 * @file tracking.h
 * @author likang (likang@freego.com)
 * @brief 跟踪
 * 包括点云数据跟踪、点云数据分割、点云数据配准等
 * @version 0.1
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <memory>
#include <functional>
#include <vector>
#include <map>
#include <chrono>
#include "clustering.h"
#include "vehicle_interface.h"

// 定义点云类型
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

// 跟踪目标结构
struct TrackedObject {
    int id;                             // 目标ID
    double predicted_x = 0.0;           // 预测x坐标
    double predicted_y = 0.0;           // 预测y坐标
    double velocity_x = 0.0;            // x方向速度
    double velocity_y = 0.0;            // y方向速度
    double width = 0.0;                 // 宽度
    double height = 0.0;                // 高度
    int age = 0;                        // 目标年龄（存在帧数）
    int missed_count = 0;               // 丢失帧数
    rclcpp::Time last_update_time;      // 最后更新时间
    ClusterInfo last_cluster;           // 最后的聚类信息
};

class Tracking {
public:
    Tracking();
    ~Tracking();
    
    // 设置跟踪结果回调函数
    void setTrackingCallback(std::function<void(const std::vector<TrackedObject>&)> callback);
    
    // 更新跟踪目标
    void updateTracking(const std::vector<ClusterInfo>& clusters, const VehicleState& vehicle_state);
    
    // 预测目标位置
    std::vector<TrackedObject> predictObjectPositions(double time_delta);
    
    // 基于车辆运动预测目标位置（雷达数据缺失时使用）
    std::vector<TrackedObject> predictObjectPositionsBasedOnVehicleMotion(const VehicleState& vehicle_state, double time_delta);
    
    // 更新轮速反馈数据
    void updateWheelSpeed(double left_wheel_speed, double right_wheel_speed);

private:
    // 数据关联 - 将聚类与现有跟踪目标关联
    std::vector<std::pair<int, int>> associateClustersWithTracks(const std::vector<ClusterInfo>& clusters);
    
    // 更新现有跟踪目标
    void updateExistingTracks(const std::vector<ClusterInfo>& clusters, const std::vector<std::pair<int, int>>& associations);
    
    // 创建新跟踪目标
    void createNewTracks(const std::vector<ClusterInfo>& clusters, const std::vector<std::pair<int, int>>& associations);
    
    // 删除丢失的跟踪目标
    void removeLostTracks();
    
    // 计算距离
    double calculateDistance(const ClusterInfo& cluster, const TrackedObject& track);
    
    // 预测目标位置（基于车辆运动）
    void predictTrackPosition(TrackedObject& track, const VehicleState& vehicle_state, double time_delta);
    
    // 跟踪结果回调函数
    std::function<void(const std::vector<TrackedObject>&)> tracking_callback_;
    
    // 跟踪目标列表
    std::vector<TrackedObject> tracked_objects_;
    
    // 下一个可用ID
    int next_object_id_;
    
    // 跟踪参数
    struct TrackingParams {
        double association_threshold = 1.0;    // 关联阈值 (m)
        int max_missed_frames = 5;             // 最大丢失帧数
        double prediction_time = 0.1;          // 预测时间间隔 (s)
    } tracking_params_;
    
    // 轮速反馈数据
    double wheel_speed_left_ = 0.0;   ///< 左轮速度 (m/s)
    double wheel_speed_right_ = 0.0;  ///< 右轮速度 (m/s)
};