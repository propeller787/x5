/**
 * @file clustering.h
 * @author likang (likang@freego.com)
 * @brief 聚类
 * 包括点云数据聚类、点云数据分割、点云数据配准等
 * @version 0.1
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <memory>
#include <functional>
#include <vector>

// 定义点云类型
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

// 聚类结果结构
struct ClusterInfo {
    PointCloud::Ptr points;        // 聚类点云
    std::vector<int> indices;      // 点云索引
    double center_x = 0.0;         // 聚类中心x坐标
    double center_y = 0.0;         // 聚类中心y坐标
    double center_z = 0.0;         // 聚类中心z坐标
    double min_x = 0.0, max_x = 0.0; // 边界框
    double min_y = 0.0, max_y = 0.0;
    double min_z = 0.0, max_z = 0.0;
    double width = 0.0;            // 宽度
    double height = 0.0;           // 高度
    double depth = 0.0;            // 深度
    int point_count = 0;           // 点数
    //所有点数z为零的数量
    int zero_z_point_count = 0;
    std::string source_name = "";
};

class Clustering : public rclcpp::Node {
public:
    Clustering();
    ~Clustering();
    
    // 设置聚类结果回调函数
    void setClusterCallback(std::function<void(const std::vector<ClusterInfo>&, const std::string&)> callback);
    
    // 对点云进行聚类处理
    void clusterPointCloud(const PointCloud::Ptr& input_cloud, const std::string& source_name);

private:
    // 使用欧几里得聚类
    std::vector<ClusterInfo> euclideanClustering(const PointCloud::Ptr& input_cloud, int min_cluster_size = -1, const std::string &source_name = "");
    
    // 计算聚类信息
    void calculateClusterInfo(ClusterInfo& cluster);
    
    // 根据车辆位置计算聚类中心点
    // 车辆中心点为后轴中心点，前车长0.85m，后车长0.25m
    // 如果在车身两侧（y轴方向），选择y轴距离短的点为中心点
    // 如果在车前或车后（x轴方向），选择x轴距离短的点为中心点
    void calculateClusterCenterByVehiclePosition(ClusterInfo& cluster);
    
    // 发布聚类后的点云数据
    void publishClusterPointCloud(const std::vector<ClusterInfo>& clusters, const std::string& source_name);
    
    // 聚类结果回调函数
    std::function<void(const std::vector<ClusterInfo>&, const std::string&)> cluster_callback_;
    
    // ROS2发布器 - 用于发布聚类后的点云数据
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_cluster_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_cluster_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr left_tilt_cluster_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr right_tilt_cluster_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr back_cluster_publisher_;
    
    // 聚类参数
    struct ClusterParams {
        double cluster_tolerance = 0.1;    // 聚类容差 (m)
        int min_cluster_size = 2;          // 最小聚类点数
        int max_cluster_size = 1000;       // 最大聚类点数
    } cluster_params_;
};