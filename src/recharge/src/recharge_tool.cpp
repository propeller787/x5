// recharge_tool.cpp
#include "recharge_tool.hpp"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <deque>

double RechargeTool::angleBetween(const IndexPoint &p1, const IndexPoint &p2)
{
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

double RechargeTool::angleDiff(double a1, double a2)
{
    double diff = fabs(a1 - a2);
    return std::min(diff, 2 * M_PI - diff) * 180.0 / M_PI;
}

double RechargeTool::averageAngle(const std::vector<double> &angles)
{
    double sin_sum = 0.0, cos_sum = 0.0;
    for (double a : angles)
    {
        sin_sum += sin(a);
        cos_sum += cos(a);
    }
    return atan2(sin_sum, cos_sum);
}

std::vector<std::vector<IndexPoint>> RechargeTool::splitPointsToSegments(
    const std::vector<IndexPoint> &points,
    double angle_threshold_deg,
    int history_window_size)
{
    std::vector<std::vector<IndexPoint>> segments;
    if (points.size() < 2)
    {
        segments.push_back(points);
        return segments;
    }

    std::vector<IndexPoint> current_segment;
    std::deque<double> angle_history;

    current_segment.push_back(points[0]);

    for (size_t i = 1; i < points.size(); i++)
    {
        const IndexPoint &p_prev = points[i - 1];
        const IndexPoint &p_curr = points[i];
        double curr_angle = angleBetween(p_prev, p_curr);

        // 更新方向历史
        angle_history.push_back(curr_angle);
        if (angle_history.size() > history_window_size)
        {
            angle_history.pop_front();
        }

        // 若历史方向足够，开始判断角度差
        if (angle_history.size() >= history_window_size)
        {
            // 计算历史方向平均值（排除当前角度）
            std::vector<double> hist(angle_history.begin(), angle_history.end() - 1);
            double avg_angle = averageAngle(hist);
            double diff = angleDiff(avg_angle, curr_angle);

            if (diff > angle_threshold_deg)
            {
                // 发生显著拐弯，分段
                current_segment.push_back(p_prev);
                segments.push_back(current_segment);

                // 开始新分段，以当前点为起点
                current_segment.clear();
                current_segment.push_back(p_curr);

                // 重置方向历史，仅保留当前角度
                angle_history.clear();
                angle_history.push_back(curr_angle);
            }
        }
    }

    current_segment.push_back(points.back());
    segments.push_back(current_segment);

    return segments;
}

std::vector<std::vector<IndexPoint>> RechargeTool::splitPointsToSegments_near(
    const std::vector<IndexPoint> &points,
    double angle_threshold_deg,
    int history_window_size)
{
    std::vector<std::vector<IndexPoint>> segments;
    if (points.size() < 2)
    {
        segments.push_back(points);
        return segments;
    }

    std::vector<IndexPoint> current_segment;
    std::deque<double> angle_history;

    current_segment.push_back(points[0]);

    const int min_count = 3;
    for (size_t i = min_count; i < points.size(); i=i+min_count)
    {
        const IndexPoint &p_prev = points[i - min_count];
        const IndexPoint &p_curr = points[i];
        double curr_angle = angleBetween(p_prev, p_curr);

        // 更新方向历史
        angle_history.push_back(curr_angle);
        if (angle_history.size() > history_window_size)
        {
            angle_history.pop_front();
        }

        // 若历史方向足够，开始判断角度差
        if (angle_history.size() >= history_window_size)
        {
            // 计算历史方向平均值（排除当前角度）
            std::vector<double> hist(angle_history.begin(), angle_history.end() - 1);
            double avg_angle = averageAngle(hist);
            double diff = angleDiff(avg_angle, curr_angle);

            if (diff > angle_threshold_deg)
            {
                // 发生显著拐弯，分段
                current_segment.push_back(p_prev);
                segments.push_back(current_segment);

                // 开始新分段，以当前点为起点
                current_segment.clear();
                current_segment.push_back(p_curr);

                // 重置方向历史，仅保留当前角度
                angle_history.clear();
                angle_history.push_back(curr_angle);
            }
        }
    }

    current_segment.push_back(points.back());
    segments.push_back(current_segment);

    return segments;
}

std::pair<double, double> RechargeTool::calculateAngleBisector(double k1, double b1, double k2, double b2)
{
    // 计算两条直线的夹角
    double angle1 = atan(k1);
    double angle2 = atan(k2);

    // 计算两条直线之间的夹角
    double delta_angle = fabs(angle1 - angle2);

    // 计算中线角度
    double bisector_angle1;
    if (angle1 < angle2)
    {
        bisector_angle1 = angle1 + delta_angle / 2;
    }
    else
    {
        bisector_angle1 = angle2 + delta_angle / 2;
    }

    // 处理平行线（无交点）的情况
    if (fabs(k1 - k2) < 1e-10)
    {
        // 平行线没有交点，返回原直线
        return {k1, b1};
    }

    // 计算交点坐标
    double x_intersect = (b2 - b1) / (k1 - k2);
    double y_intersect = k1 * x_intersect + b1;

    // 计算平分线的斜率
    double k3 = tan(bisector_angle1);
    double b3 = y_intersect - k3 * x_intersect;

    return {k3, b3};
}

std::pair<double, double> RechargeTool::computeDockingVelocity(double dx1, double dy1, double yaw_error1)
{
    double dx = dx1;
    double dy = dy1;
    double yaw_error = yaw_error1;
    {
        double a = (dx1+0.0) * tan(yaw_error1);
        dy = 0.0*a + dy1;
    }
    double max_linear_speed = 0.14;
    double max_angular_speed = 0.18;

    double Kp_linear = 0.5;
    double Kp_angular = 0.2;
    double Kp_lateral = 0.8;

    if (dx > 1.8 - 0.2) {
        Kp_angular = 0.3;
        Kp_lateral = 0.7;
    } else if (dx > 1.3 - 0.2) { // 远距离 dy优先
        Kp_angular = 0.4;
        Kp_lateral = 0.7;
    } else if (dx > 1.1 -0.2) { //中等距离 平衡
        Kp_angular = 0.4;
        Kp_lateral = 0.7;
    } else if (dx > 0.95 -0.2) { //中等距离 平衡
        Kp_angular = 0.5;
        Kp_lateral = 0.5;
    } else if (dx > 0.85 -0.2) { //近距离 yaw优先
        Kp_angular = 0.8;
        Kp_lateral = 0.2;
    }

    // 线速度控制
    double linear_velocity = Kp_linear * std::max(0.1, dx - 0.6);
    linear_velocity = std::clamp(linear_velocity, -max_linear_speed, max_linear_speed);
    linear_velocity = std::max(0.08, linear_velocity);

    // 角速度控制
    double angular_velocity = Kp_angular * yaw_error + Kp_lateral * dy;
    angular_velocity = std::clamp(angular_velocity, -max_angular_speed, max_angular_speed);


    float angular_velocity_abs = std::abs(angular_velocity);
    float radio = 1.0;
    if ((dx > 0.60 - 0.2) && std::abs(yaw_error1) > 0.025) {
        if (angular_velocity_abs >= 0.1) {
            radio = (0.19 - angular_velocity_abs)/ 0.20;
        } else if(angular_velocity_abs > 0.03){
            radio = (0.11 - angular_velocity_abs)/ 0.12;
        }
    } else if (std::abs(dy) > 0.07) {
        if (angular_velocity_abs >= 0.1) {
            radio = (0.19 - angular_velocity_abs)/ 0.20;
        } else if(angular_velocity_abs > 0.03){
            radio = (0.11 - angular_velocity_abs)/ 0.12;
        }
    }
    // printf("radio:%.2f\n", radio);
    linear_velocity = (0.7 * linear_velocity) + (0.3 * radio * linear_velocity);

    return {-linear_velocity, -angular_velocity};
}

std::tuple<double, double, double> RechargeTool::simpleFitLine(const std::vector<IndexPoint> &points)
{
    if (points.size() < 2)
    {
        return {0.0, 0.0, 0.0};
    }

    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;
    int n = points.size();

    for (const auto &p : points)
    {
        sum_x += p.x;
        sum_y += p.y;
        sum_xy += p.x * p.y;
        sum_xx += p.x * p.x;
    }

    double k = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    double b = (sum_y - k * sum_x) / n;
    
    // 使用首尾点计算方向
    double dx = points.back().x - points.front().x;
    double dy = points.back().y - points.front().y;
    double yaw = atan2(dy, dx);
    
    return {k, b, yaw};
}
