#pragma once
#include "control.h"
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace service::control {

/**
 * @brief 车辆召唤控制类，继承自基础控制类
 * 
 * 实现车辆召唤功能的核心逻辑，包括：
 * - GPS坐标点匹配
 * - UTM坐标转换
 * - 召唤服务请求处理
 * - 最近点搜索算法
 */
class SummonControl : public Control {

public:
    SummonControl();

private:
    /**
     * @brief 处理召唤服务请求回调函数
     * @param request 召唤请求参数，包含目标GPS坐标等信息
     * @param response 响应对象，返回操作结果状态码
     */
    void handleSummon(const std::shared_ptr<interface::srv::Summon::Request> request,
        std::shared_ptr<interface::srv::Summon::Response> response);
        
    /**
     * @brief 处理召唤状态查询请求
     * @param request 状态查询请求
     * @param response 返回当前召唤状态
     */
    void handleSummonStatus(const std::shared_ptr<interface::srv::SummonStatus::Request> request,
        std::shared_ptr<interface::srv::SummonStatus::Response> response);
        
    /**
     * @brief 从文件加载预定义的GPS坐标点列表
     * @param filename GPS坐标点数据文件路径，格式为经度,纬度,海拔
     */
    void loadGPSPoints(const std::string &filename);
    
    /**
     * @brief 在预存GPS点列表中查找距离给定点最近的点
     * @param point 需要匹配的GPS坐标点
     * @return 最近邻的GPS坐标点
     */
    sensor_msgs::msg::NavSatFix get_nearest_point(const sensor_msgs::msg::NavSatFix& point);

private:
    // ROS2服务相关
    rclcpp::Service<interface::srv::Summon>::SharedPtr car_summon_ { nullptr };       ///< 召唤车辆服务(处理客户端召唤请求)
    rclcpp::Service<interface::srv::SummonStatus>::SharedPtr car_summon_status_ { nullptr };  ///< 召唤状态查询服务

    // UTM坐标参数（默认值对应深圳坪山具体位置）
    int utm_zone_ = 50;           ///< UTM投影带编号（中国东部地区常用50带）
    double utm_x_ = 424850.658654143;  ///< 基准点东坐标（单位：米）
    double utm_y_ = 4416184.503079025; ///< 基准点北坐标（单位：米）

    std::vector<sensor_msgs::msg::NavSatFix> gps_points_;  ///< 预存储的GPS坐标点列表（用于位置匹配）
};

}