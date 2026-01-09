/**
 * @file robot_dimensions.h
 * @brief 机器人主体尺寸与碰撞检测区域配置
 */
#pragma once

struct RobotDimensions {
    struct BaseDimensions {
        double length = 0.90; // 前后方向尺寸，对应URDF base_link 碰撞几何体长度
        double width = 0.54;  // 左右方向尺寸，对应URDF base_link 碰撞几何体宽度
        double height = 0.8;  // 上下方向尺寸

        // 安全边距（可根据实车调参）
        double safety_margin_front = 0.10;
        double safety_margin_back = 0.20;
        double safety_margin_side = 0.08;
    };

    struct CollisionZones {
        struct FrontZone {
            double min_x = 0.0;   // 机器人前缘
            double max_x = 1.5;   // 前向检测最远距离
            double min_y = -0.32; // 机器人左缘（半宽）
            double max_y = 0.32;  // 机器人右缘（半宽）
        } front_zone;

        struct BackZone {
            double min_x = -1.5; // 后向检测最远距离
            double max_x = 0.0;  // 机器人后缘
            double min_y = -0.32;
            double max_y = 0.32;
        } back_zone;
    };

    static BaseDimensions getBaseDimensions() { return BaseDimensions(); }
    static CollisionZones getCollisionZones() { return CollisionZones(); }

    static bool isPointInFrontZone(double x, double y, const CollisionZones &zones) {
        return (x >= zones.front_zone.min_x && x <= zones.front_zone.max_x && y >= zones.front_zone.min_y && y <= zones.front_zone.max_y);
    }
    static bool isPointInBackZone(double x, double y, const CollisionZones &zones) {
        return (x >= zones.back_zone.min_x && x <= zones.back_zone.max_x && y >= zones.back_zone.min_y && y <= zones.back_zone.max_y);
    }
};
