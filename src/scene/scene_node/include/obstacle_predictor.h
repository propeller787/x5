/**
 * @file obstacle_predictor.h
 * @author likang (likang@freego.com)
 * @brief 障碍物位置预测器 - 基于轮速反馈的实时运动补偿
 *
 * 设计动机:
 *   雷达扫描频率(~10Hz)远低于轮速反馈频率(~50Hz)。在两次雷达更新之间,
 *   车辆已发生位移/转向,但 radar_cluster_map_ 中的障碍物位置仍是旧帧坐标。
 *   本模块在每次收到轮速反馈时,利用 v/omega 实时补偿所有已缓存的障碍物位置,
 *   使决策器始终能拿到"当前车体坐标系下"最新的障碍物估计。
 *
 * 核心思路（车体坐标系运动补偿）:
 *   假设障碍物在地面坐标系静止,车辆以 (v, omega) 运动 dt 秒:
 *     dtheta = -omega * dt          (车左转 → 障碍物在车体系中右转)
 *     dx     = -v * dt              (车前进 → 障碍物在车体系中后退)
 *     new_x  = x * cos(dtheta) - y * sin(dtheta) + dx
 *     new_y  = x * sin(dtheta) + y * cos(dtheta)
 *
 * 数据流:
 *   Clustering ──(雷达帧)──► updateRadarClusters()   // 存入新鲜聚类
 *   VehicleInterface ──(轮速)──► onWheelSpeedUpdate() // 增量补偿
 *   DecisionMaker ──(决策时)──► getPredictedClusters() // 读取补偿后数据
 *
 * @version 0.1
 * @date 2026-04-06
 * @copyright Copyright (c) 2026 Freego Technology Co., Ltd.
 */
#pragma once

#include "clustering.h"
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

/**
 * @struct ObstaclePredictorParams
 * @brief 预测器可调参数（通过 YAML / ROS 参数加载）
 */
struct ObstaclePredictorParams {
    bool enable = true;                       ///< 总开关：false 时 getPredictedClusters 直接返回原始数据
    double max_prediction_age = 0.5;          ///< 累计补偿时间上限(s)，超过则认为预测不可靠，丢弃该雷达数据
    double max_distance_drift_ratio = 0.5;    ///< 补偿后距离与原始距离偏差超过 ratio*原距离+1m 则丢弃
    double min_valid_x = -1.0;                ///< 补偿后 center_x 低于此值则丢弃（目标移到车后方）
};

/**
 * @class ObstaclePredictor
 * @brief 轮速驱动的障碍物位置实时补偿器
 *
 * 线程安全: 所有公共方法均通过 mutex_ 保护，可在不同回调线程中安全调用。
 */
class ObstaclePredictor {
public:
    explicit ObstaclePredictor(rclcpp::Logger logger);

    /**
     * @brief 更新预测器参数（支持运行时热更新）
     */
    void updateParams(const ObstaclePredictorParams& params);

    /**
     * @brief 雷达帧到达时调用 - 存入新鲜聚类并重置该雷达的累计补偿
     * @param clusters 本帧聚类结果
     * @param radar_name 雷达源名称 (如 "left", "right", "back")
     * @param timestamp 雷达帧时间戳
     */
    void updateRadarClusters(const std::vector<ClusterInfo>& clusters,
                             const std::string& radar_name,
                             const rclcpp::Time& timestamp);

    /**
     * @brief 轮速反馈到达时调用 - 对所有已缓存雷达的聚类做增量运动补偿
     * @param linear_velocity 线速度 v (m/s, 前进为正)
     * @param angular_velocity 角速度 omega (rad/s, 左转为正)
     * @param timestamp 轮速帧时间戳
     */
    void onWheelSpeedUpdate(double linear_velocity, double angular_velocity,
                            const rclcpp::Time& timestamp);

    /**
     * @brief 获取指定雷达当前补偿后的聚类数据
     * @param radar_name 雷达源名称
     * @return 补偿后的聚类列表（若关闭或无数据则返回空）
     */
    std::vector<ClusterInfo> getPredictedClusters(const std::string& radar_name) const;

    /**
     * @brief 获取所有雷达补偿后的聚类数据（按雷达名索引）
     */
    std::map<std::string, std::vector<ClusterInfo>> getAllPredictedClusters() const;

    /**
     * @brief 获取指定雷达数据的年龄（距最后一次雷达更新的秒数）
     * @return 秒数；若无数据返回极大值
     */
    double getRadarDataAge(const std::string& radar_name, const rclcpp::Time& now) const;

private:
    /// 原始聚类中心坐标（与 clusters 保持 1:1 索引对齐）
    struct OriginalCenter {
        double x = 0.0;
        double y = 0.0;
    };

    /**
     * @brief 对单个雷达的所有聚类施加一次增量补偿
     * @param clusters 待补偿的聚类（就地修改）
     * @param original_centers 每个聚类的原始中心（与 clusters 1:1 对齐，就地同步删除）
     * @param dt 时间增量 (s)
     * @param linear_vel 线速度 (m/s)
     * @param angular_vel 角速度 (rad/s)
     */
    void compensateClusters(std::vector<ClusterInfo>& clusters,
                            std::vector<OriginalCenter>& original_centers,
                            double dt, double linear_vel, double angular_vel) const;

    /**
     * @brief 对单个聚类施加运动补偿
     * @param orig_x 该聚类在原始雷达帧中的 center_x（用于累计偏差校验）
     * @param orig_y 该聚类在原始雷达帧中的 center_y
     * @return true 表示补偿结果有效，false 表示应丢弃
     */
    bool compensateOneCluster(ClusterInfo& cluster,
                              double cos_theta, double sin_theta,
                              double vehicle_dx,
                              double orig_x, double orig_y) const;

    /// 每个雷达的缓存数据
    struct RadarCache {
        std::vector<ClusterInfo> clusters;              ///< 当前已补偿的聚类
        std::vector<OriginalCenter> original_centers;   ///< 每个聚类在雷达帧中的原始中心（1:1 对应 clusters）
        rclcpp::Time radar_timestamp;                   ///< 雷达帧时间戳
        double accumulated_dt = 0.0;                    ///< 自该雷达帧以来的累计补偿时间
    };

    mutable std::mutex mutex_;
    rclcpp::Logger logger_;
    ObstaclePredictorParams params_;
    std::map<std::string, RadarCache> radar_caches_;
    rclcpp::Time last_wheel_speed_time_;
    bool has_wheel_speed_ = false;
};
