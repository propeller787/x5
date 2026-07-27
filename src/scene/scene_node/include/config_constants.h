/**
 * @file config_constants.h
 * @author likang (likang@freego.com)
 * @brief 系统配置常量定义 - 集中管理所有配置参数和魔法数字
 * 
 * 设计目的:
 * 1. 集中管理系统配置，提高可维护性
 * 2. 避免魔法数字分散在代码中
 * 3. 提供清晰的配置说明和单位
 * 4. 便于性能调优和参数调整
 * 
 * @version 0.1
 * @date 2025-10-28
 * @copyright Copyright (c) 2025 Freego Technology Co., Ltd.
 */

#pragma once

#include <cstddef>

namespace scene_node {

// ==================== 系统频率配置 ====================

/**
 * @brief 雷达数据采集频率 (Hz)
 * 
 * 说明: 雷达数据的更新频率，决定了障碍物检测的时间分辨率
 * 推荐值: 10-20 Hz (过高会增加计算负担，过低会降低响应速度)
 */
constexpr double RADAR_FREQUENCY = 10.0;

/**
 * @brief 控制命令输出频率 (Hz)
 * 
 * 说明: 控制命令的发布频率，应远高于雷达频率以保证控制平滑性
 * 推荐值: 50-100 Hz
 */
constexpr double CONTROL_FREQUENCY = 70.0;

/**
 * @brief 雷达数据超时时间 (秒)
 * 
 * 说明: 超过此时间未收到雷达数据视为超时，启用预测模式
 * 推荐值: 1.5 * (1/RADAR_FREQUENCY) = 0.15s
 */
constexpr double RADAR_TIMEOUT = 0.05;

// ==================== 距离与速度限制 ====================

/**
 * @brief 最小距离阈值 (米)
 * 
 * 说明: 障碍物在此距离内触发紧急制动
 * 安全考虑: 应考虑车辆制动距离和反应时间
 */
constexpr double MIN_DISTANCE_THRESHOLD = 0.3;

/**
 * @brief 最大距离阈值 (米)
 * 
 * 说明: 超过此距离不进行速度限制
 * 性能考虑: 过大会导致频繁限速，过小会降低安全性
 */
constexpr double MAX_DISTANCE_THRESHOLD = 1.1;

/**
 * @brief 紧急制动距离 (米)
 * 
 * 说明: 在此距离内立即制动 (速度限制为0)
 * 应与MIN_DISTANCE_THRESHOLD保持一致
 */
constexpr double EMERGENCY_BRAKE_DISTANCE = 0.3;

/**
 * @brief 最小速度限制 (m/s)
 * 
 * 换算: 0.056 m/s ≈ 0.2 km/h
 * 说明: 障碍物很近时允许的最大速度
 */
constexpr double MIN_SPEED_LIMIT = 0.056;

/**
 * @brief 最大速度限制 (m/s)
 * 
 * 换算: 2.22 m/s ≈ 8.0 km/h
 * 说明: 障碍物较远时允许的最大速度
 */
constexpr double MAX_SPEED_LIMIT = 2.22;

// ==================== 聚类参数配置 ====================

/**
 * @brief 聚类容差 (米)
 * 
 * 说明: 欧几里得聚类算法的距离阈值
 * - 过小: 会将同一物体分成多个聚类
 * - 过大: 会将不同物体合并为一个聚类
 */
constexpr double CLUSTER_TOLERANCE = 0.5;

/**
 * @brief 最小聚类点数
 * 
 * 说明: 少于此点数的聚类将被过滤为噪声
 * 推荐值: 2-5点
 */
constexpr int MIN_CLUSTER_SIZE = 2;

/**
 * @brief 最大聚类点数
 * 
 * 说明: 超过此点数的聚类可能是多个物体重叠
 * 推荐值: 500-2000点
 */
constexpr int MAX_CLUSTER_SIZE = 1000;

// ==================== 跟踪参数配置 ====================

/**
 * @brief 数据关联阈值 (米)
 * 
 * 说明: 新检测与已有跟踪目标的最大匹配距离
 * - 过小: 容易丢失目标
 * - 过大: 容易错误关联
 */
constexpr double ASSOCIATION_THRESHOLD = 1.0;

/**
 * @brief 最大丢失帧数
 * 
 * 说明: 连续多少帧未匹配到目标后删除该跟踪
 * 推荐值: 3-10帧 (根据雷达频率调整)
 */
constexpr int MAX_MISSED_FRAMES = 5;

/**
 * @brief 目标位置预测时间间隔 (秒)
 * 
 * 说明: 预测目标未来位置的时间步长
 * 推荐值: 1/RADAR_FREQUENCY = 0.1s
 */
constexpr double PREDICTION_TIME = 0.1;

// ==================== 平滑滤波器配置 ====================

/**
 * @brief 滤波器窗口大小
 * 
 * 说明: 移动平均滤波使用的历史数据点数
 * - 过小: 滤波效果不明显
 * - 过大: 响应延迟增加
 */
constexpr size_t FILTER_WINDOW_SIZE = 5;

/**
 * @brief 指数平滑系数 (Alpha)
 * 
 * 说明: 指数平滑滤波器的权重系数
 * - 接近0: 更平滑，但响应慢
 * - 接近1: 响应快，但不够平滑
 * 推荐值: 0.3-0.7
 */
constexpr double EXPONENTIAL_ALPHA = 0.5;

// ==================== ROS2通信配置 ====================

/**
 * @brief ROS2订阅/发布队列大小
 */
constexpr int QUEUE_SIZE = 10;

/**
 * @brief 日志节流时间 (毫秒)
 * 
 * 说明: 相同日志输出的最小时间间隔，避免日志刷屏
 */
constexpr int LOG_THROTTLE_MS = 5000;

// ==================== 历史数据管理 ====================

/**
 * @brief 最大历史数据大小
 * 
 * 说明: 保存的历史数据帧数，用于数据分析和预测
 * 推荐值: 10-50帧
 */
constexpr size_t MAX_HISTORY_SIZE = 10;

// ==================== 性能调优参数 ====================

/**
 * @brief 点云降采样体素大小 (米)
 * 
 * 说明: 体素滤波器的叶子大小
 * - 过小: 点云密集，计算量大
 * - 过大: 丢失细节信息
 */
constexpr double VOXEL_LEAF_SIZE = 0.05;

/**
 * @brief 统计滤波邻近点数
 * 
 * 说明: 统计离群点检测的K近邻数量
 */
constexpr int STATISTICAL_MEAN_K = 20;

/**
 * @brief 统计滤波标准差倍数
 * 
 * 说明: 超过均值多少倍标准差视为离群点
 * 推荐值: 1.0-2.0
 */
constexpr double STATISTICAL_STD_DEV = 1.0;

// ==================== 单位转换辅助函数 ====================

/**
 * @brief 将km/h转换为m/s
 * @param kmh 速度 (km/h)
 * @return double 速度 (m/s)
 */
constexpr double kmhToMs(double kmh) {
    return kmh / 3.6;
}

/**
 * @brief 将m/s转换为km/h
 * @param ms 速度 (m/s)
 * @return double 速度 (km/h)
 */
constexpr double msToKmh(double ms) {
    return ms * 3.6;
}

/**
 * @brief 将角度转换为弧度
 * @param degrees 角度 (度)
 * @return double 角度 (弧度)
 */
constexpr double degreesToRadians(double degrees) {
    return degrees * 3.14159265358979323846 / 180.0;
}

/**
 * @brief 将弧度转换为角度
 * @param radians 角度 (弧度)
 * @return double 角度 (度)
 */
constexpr double radiansToDegrees(double radians) {
    return radians * 180.0 / 3.14159265358979323846;
}

} // namespace scene_node

// ==================== 配置验证 ====================

/**
 * @brief 编译期配置参数验证
 * 
 * 确保配置参数符合逻辑约束
 */
namespace scene_node {
namespace validation {
    // 距离参数验证
    static_assert(MIN_DISTANCE_THRESHOLD > 0.0, 
        "最小距离阈值必须大于0");
    static_assert(MAX_DISTANCE_THRESHOLD > MIN_DISTANCE_THRESHOLD, 
        "最大距离阈值必须大于最小距离阈值");
    static_assert(EMERGENCY_BRAKE_DISTANCE >= MIN_DISTANCE_THRESHOLD,
        "紧急制动距离不应小于最小距离阈值");
    
    // 速度参数验证
    static_assert(MIN_SPEED_LIMIT >= 0.0, 
        "最小速度限制必须非负");
    static_assert(MAX_SPEED_LIMIT > MIN_SPEED_LIMIT, 
        "最大速度限制必须大于最小速度限制");
    
    // 频率参数验证
    static_assert(RADAR_FREQUENCY > 0.0, 
        "雷达频率必须大于0");
    static_assert(CONTROL_FREQUENCY > RADAR_FREQUENCY, 
        "控制频率应高于雷达频率");
    
    // 聚类参数验证
    static_assert(CLUSTER_TOLERANCE > 0.0, 
        "聚类容差必须大于0");
    static_assert(MIN_CLUSTER_SIZE > 0, 
        "最小聚类大小必须大于0");
    static_assert(MAX_CLUSTER_SIZE > MIN_CLUSTER_SIZE, 
        "最大聚类大小必须大于最小聚类大小");
    
    // 滤波器参数验证
    static_assert(EXPONENTIAL_ALPHA > 0.0 && EXPONENTIAL_ALPHA < 1.0, 
        "指数平滑系数必须在(0,1)区间内");
    static_assert(FILTER_WINDOW_SIZE > 0, 
        "滤波器窗口大小必须大于0");
}
} // namespace scene_node::validation

