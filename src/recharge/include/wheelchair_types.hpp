// wheelchair_types.hpp
#ifndef WHEELCHAIR_TYPES_HPP
#define WHEELCHAIR_TYPES_HPP

#include <vector>
#include <string>
#include <Eigen/Dense>

// 轮椅状态枚举
enum class WheelchairState
{
    READY,     // 就绪中
    WALKING,   // 行走中
    ROTATING,  // 旋转中
    SEARCHING, // 搜索中
    CHARGING   // 充电中
};

// 充电状态枚举
enum class ChargingState
{
    IDLE,
    NAVIGATING,
    CHARGING,
    UNAVAILABLE,
    COMPLETED
};

// 电池状态枚举
enum class BatteryState
{
    NORMAL,   // 正常
    LOW,      // 低电量
    CRITICAL, // 严重低电量
    CHARGING, // 充电中
    FULL      // 充满
};

// ==================== 数据结构 ====================
struct IndexPoint
{
    double x;
    double y;
    int index;

    IndexPoint(double x = 0.0, double y = 0.0, int index = 0)
        : x(x), y(y), index(index) {}
};

struct StraightLine
{
    double k;
    double b;
    double yaw; // 方向角（弧度），范围 [-π, π)

    StraightLine(double k = 0.0, double b = 0.0, double yaw = 0.0) : k(k), b(b), yaw(yaw) {}
};


struct ScanFilterParam //奥比基站参数
{
    int min_index;
    int max_index;
    double min_intensities;
    double max_intensities;
    double min_distance;
    double max_distance;
    int min_count;
    double dock_length;
    int baselink_back_index;
    double dy_diff;
    int fixed_index;
    double scan_angle_diff;
};

struct RechargePose
{
    double dx;
    double dy;
    double dy_center;
    double k;
    double yaw_diff;
};

struct RechargeResult
{
    int frameid;
    std::vector<IndexPoint> points;
    std::vector<IndexPoint> left_points;
    std::vector<IndexPoint> right_points;
    IndexPoint middle_point;
    StraightLine middle_line;
    StraightLine left_line;
    StraightLine right_line;
    RechargePose rechargePose;
};

#endif // WHEELCHAIR_TYPES_HPP