// recharge_tool.hpp
#ifndef RECHARGE_TOOL_HPP
#define RECHARGE_TOOL_HPP

#include "wheelchair_types.hpp"
#include <vector>
#include <Eigen/Dense>

class RechargeTool
{
public:
    static double angleBetween(const IndexPoint &p1, const IndexPoint &p2);
    static double angleDiff(double a1, double a2);
    static double averageAngle(const std::vector<double> &angles);

    static std::vector<std::vector<IndexPoint>> splitPointsToSegments(
        const std::vector<IndexPoint> &points,
        double angle_threshold_deg = 15.0,
        int history_window_size = 3);

    static std::vector<std::vector<IndexPoint>> splitPointsToSegments_near(
        const std::vector<IndexPoint> &points,
        double angle_threshold_deg = 15.0,
        int history_window_size = 3);

    static std::pair<double, double> calculateAngleBisector(double k1, double b1, double k2, double b2);
    static std::pair<double, double> computeDockingVelocity(double dx, double dy, double yaw_error);

    // 直线拟合（简化版，用于基础功能）
    static std::tuple<double, double, double> simpleFitLine(const std::vector<IndexPoint> &points);
};

#endif // RECHARGE_TOOL_HPP