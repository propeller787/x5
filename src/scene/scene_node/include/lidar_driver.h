/**
 * @file lidar_driver.h
 * @author likang (likang@freego.com)
 * @brief 接收雷达数据并处理ROI区域
 * @version 0.1
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <memory>
#include <functional>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

// 定义点云类型
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

// ROI区域结构体
struct ROIBox {
    double min_x, max_x;  // 前后范围
    double min_y, max_y;  // 左右范围
    double min_z, max_z;  // 上下范围
};

// ROI区域类型枚举
enum class ROIRegionType {
    FORWARD,    // 前向区域
    LEFT,       // 左侧区域
    RIGHT,      // 右侧区域
    BACKWARD    // 后向区域
};

class LidarDriver : public rclcpp::Node {
public:
    LidarDriver();
    ~LidarDriver();

    // 设置回调函数，用于处理接收到的点云数据
    void setPointCloudCallback(std::function<void(const PointCloud::Ptr&, const std::string&)> callback);

private:
    // 雷达数据回调函数
    void leftLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void rightLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void leftTiltPointCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void rightTiltPointCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void backLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    
    // 将LaserScan转换为PointCloud
    PointCloud::Ptr laserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg, const std::string& frame_id);
    
    // 将PointCloud2转换为PCL点云
    PointCloud::Ptr pointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);
    
    // 坐标系转换 - 将所有雷达数据转换为base_link坐标系
    PointCloud::Ptr transformToBaseLink(const PointCloud::Ptr& input_cloud, const std::string& source_frame);
    
    // 坐标系转换 - 转换为base_link坐标系但不转换z轴（保留原始z值）
    PointCloud::Ptr transformToBaseLinkWithoutZ(const PointCloud::Ptr& input_cloud, const std::string& source_frame);
    
    // ROI区域过滤
    PointCloud::Ptr applyROIFilter(const PointCloud::Ptr& input_cloud, const ROIBox& roi_box);
    
    // 根据ROI区域类型过滤点云
    PointCloud::Ptr applyROIFilterByType(const PointCloud::Ptr& input_cloud, ROIRegionType region_type);
    
    // 对所有ROI区域进行过滤，返回过滤后的点云映射
    std::map<ROIRegionType, PointCloud::Ptr> applyAllROIFilters(const PointCloud::Ptr& input_cloud);
    
    // 初始化ROI区域配置
    void initializeROIRegions();
    
    // 获取指定ROI区域配置
    ROIBox getROIRegion(ROIRegionType region_type) const;
    
    // 获取ROI区域类型名称
    std::string getRegionTypeName(ROIRegionType region_type) const;

    // Z方向过滤 - 过滤掉z小于阈值的点
    PointCloud::Ptr filterByZThreshold(const PointCloud::Ptr& input_cloud, double z_threshold);

    // 发布base_link坐标系转换后的点云数据
    void publishBaseLinkPointCloud(const PointCloud::Ptr& transformed_cloud, const std::string& source_name);
    
private:
    // ROS2订阅器
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr left_laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr right_laser_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_tilt_point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_tilt_point_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr back_laser_sub_;
    
    // ROS2发布器 - 用于发布base_link坐标系转换后的点云数据
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_laser_base_link_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_laser_base_link_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_tilt_base_link_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_tilt_base_link_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr back_laser_base_link_publisher_;
    
    // TF2相关
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
     
    // 点云数据回调函数
    std::function<void(const PointCloud::Ptr&, const std::string&)> pointcloud_callback_;
    
    // ROI区域配置
    std::map<ROIRegionType, ROIBox> roi_regions_;
    
    // Z方向过滤阈值（米）- 过滤掉z小于此值的点
    double tilt_z_filter_threshold_;
};