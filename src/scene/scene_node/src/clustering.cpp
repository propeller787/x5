/**
 * @file clustering.cpp
 * @author likang (likang@freego.com)
 * @brief 聚类实现
 * @version 0.1
 * @date 2025-10-16
 *
 * @copyright Copyright (c) 2025
 */

#include "clustering.h"
#include "../../../common/utils/topic.h"
#include <algorithm>
#include <geometry_msgs/msg/pose.hpp>
#include <iostream>
#include <limits>
#include <pcl_conversions/pcl_conversions.h>

Clustering::Clustering() : rclcpp::Node("clustering_node") {
    RCLCPP_INFO(this->get_logger(), "初始化聚类模块");

    // 创建发布器 - 用于发布聚类后的点云数据
    RCLCPP_INFO(this->get_logger(), "创建左雷达聚类点云发布器，话题: %s", leftLaserClusterTopic);
    left_cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(leftLaserClusterTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建右雷达聚类点云发布器，话题: %s", rightLaserClusterTopic);
    right_cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(rightLaserClusterTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建左倾斜雷达聚类点云发布器，话题: /laser/left_tilt_cluster");
    left_tilt_cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(leftTiltClusterTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建右倾斜雷达聚类点云发布器，话题: /laser/right_tilt_cluster");
    right_tilt_cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(rightTiltClusterTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建后雷达聚类点云发布器，话题: /laser/back_cluster");
    back_cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/laser/back_cluster", 10);
}

Clustering::~Clustering() { RCLCPP_INFO(this->get_logger(), "聚类模块析构"); }

void Clustering::setClusterCallback(std::function<void(const std::vector<ClusterInfo> &, const std::string &)> callback) { cluster_callback_ = callback; }

void Clustering::clusterPointCloud(const PointCloud::Ptr &input_cloud, const std::string &source_name) {
    if (!input_cloud || input_cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "输入点云为空，跳过聚类");
        return;
    }

    RCLCPP_DEBUG(this->get_logger(), "开始对 %s 雷达数据进行聚类，输入点数: %zu", source_name.c_str(), input_cloud->points.size());

    // 根据源名称确定最小聚类点数：left_tilt 和 right_tilt 使用1，其他使用默认值2
    int min_cluster_size = cluster_params_.min_cluster_size;
    if (source_name == "left_tilt" || source_name == "right_tilt") {
        min_cluster_size = 1;
    }

    // 执行欧几里得聚类
    auto clusters = euclideanClustering(input_cloud, min_cluster_size, source_name);

    RCLCPP_DEBUG(this->get_logger(), "聚类完成，检测到 %zu 个聚类", clusters.size());

    // 发布聚类后的点云数据
    if (!clusters.empty()) {
        publishClusterPointCloud(clusters, source_name);
    }

    // 触发回调函数
    if (cluster_callback_ && !clusters.empty()) {
        cluster_callback_(clusters, source_name);
    }
}

std::vector<ClusterInfo> Clustering::euclideanClustering(const PointCloud::Ptr &input_cloud, int min_cluster_size, const std::string &source_name) {
    std::vector<ClusterInfo> clusters;

    // 创建KD树
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(input_cloud);

    // 如果未指定最小聚类点数，使用默认值
    if (min_cluster_size < 0) {
        min_cluster_size = cluster_params_.min_cluster_size;
    }

    // 创建聚类提取器
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setInputCloud(input_cloud);
    ec.setClusterTolerance(cluster_params_.cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(cluster_params_.max_cluster_size);
    ec.setSearchMethod(tree);

    // 执行聚类
    std::vector<pcl::PointIndices> cluster_indices;
    ec.extract(cluster_indices);

    // 处理聚类结果
    for (const auto &indices : cluster_indices) {
        ClusterInfo cluster_info;
        cluster_info.points = std::make_shared<PointCloud>();
        cluster_info.indices = indices.indices;

        // 提取聚类点云
        for (int index : indices.indices) {
            cluster_info.points->points.push_back(input_cloud->points[index]);
        }

        cluster_info.points->width = cluster_info.points->points.size();
        cluster_info.points->height = 1;
        cluster_info.points->header = input_cloud->header;

        cluster_info.point_count = cluster_info.points->points.size();

        // 计算聚类信息
        // calculateClusterInfo(cluster_info);
        calculateClusterCenterByVehiclePosition(cluster_info);
        cluster_info.source_name = source_name;
        clusters.push_back(cluster_info);
    }
    return clusters;
}

void Clustering::calculateClusterInfo(ClusterInfo &cluster) {
    if (cluster.points->points.empty()) {
        return;
    }

    // 初始化边界值
    cluster.min_x = cluster.max_x = cluster.points->points[0].x;
    cluster.min_y = cluster.max_y = cluster.points->points[0].y;

    // 车辆位置（原点）
    const double vehicle_x = 0.0;
    const double vehicle_y = 0.0;

    // 找到距离车辆最近的点作为中心点
    double min_distance_sq = std::numeric_limits<double>::max();
    int nearest_point_idx = 0;

    // 计算中心点和边界框
    for (size_t i = 0; i < cluster.points->points.size(); ++i) {
        const auto &point = cluster.points->points[i];
        
        // 更新边界框
        cluster.min_x = std::min(cluster.min_x, (double)point.x);
        cluster.max_x = std::max(cluster.max_x, (double)point.x);
        cluster.min_y = std::min(cluster.min_y, (double)point.y);
        cluster.max_y = std::max(cluster.max_y, (double)point.y);
        cluster.min_z = std::min(cluster.min_z, (double)point.z);
        cluster.max_z = std::max(cluster.max_z, (double)point.z);
        if (std::abs(point.z) < 0.01) {
            cluster.zero_z_point_count++;
        }
        // 计算到车辆（原点）的距离的平方
        double dx = point.x - vehicle_x;
        double dy = point.y - vehicle_y;
        double distance_sq = dx * dx + dy * dy;

        // 更新最近点
        if (distance_sq < min_distance_sq) {
            min_distance_sq = distance_sq;
            nearest_point_idx = i;
        }
    }

    // 使用距离车辆最近的点作为中心点
    cluster.center_x = cluster.points->points[nearest_point_idx].x;
    cluster.center_y = cluster.points->points[nearest_point_idx].y;
    cluster.center_z = cluster.points->points[nearest_point_idx].z;
    // 计算尺寸
    cluster.width = cluster.max_x - cluster.min_x;
    cluster.height = cluster.max_y - cluster.min_y;

    RCLCPP_DEBUG(this->get_logger(), "聚类信息: 中心(%.3f, %.3f, %.3f), 尺寸(%.3f x %.3f), 点数: %d", cluster.center_x, cluster.center_y, cluster.center_z, cluster.width,
                 cluster.height, cluster.point_count);
}

void Clustering::calculateClusterCenterByVehiclePosition(ClusterInfo &cluster) {
    if (cluster.points->points.empty()) {
        return;
    }

    // 车辆中心点为后轴中心点（原点）
    const double vehicle_x = 0.0;
    const double vehicle_y = 0.0;
    
    // 车辆尺寸：中心点前车长0.85m，中心点后车长0.25m
    const double vehicle_front_length = 0.85;  // 前车长
    const double vehicle_back_length = 0.25;   // 后车长
    
    // 初始化边界值
    cluster.min_x = cluster.max_x = cluster.points->points[0].x;
    cluster.min_y = cluster.max_y = cluster.points->points[0].y;
    cluster.min_z = cluster.max_z = cluster.points->points[0].z;
    // 统计边界框和判断聚类位置
    int points_in_vehicle_side = 0;  // 在车身两侧的点数
    int points_in_front_back = 0;    // 在车前或车后的点数
    
    for (size_t i = 0; i < cluster.points->points.size(); ++i) {
        const auto &point = cluster.points->points[i];
        
        // 更新边界框
        cluster.min_x = std::min(cluster.min_x, (double)point.x);
        cluster.max_x = std::max(cluster.max_x, (double)point.x);
        cluster.min_y = std::min(cluster.min_y, (double)point.y);
        cluster.max_y = std::max(cluster.max_y, (double)point.y);
        cluster.min_z = std::min(cluster.min_z, (double)point.z);
        cluster.max_z = std::max(cluster.max_z, (double)point.z);
        if (std::abs(point.z) < 0.00001) {
            cluster.zero_z_point_count++;
        }
        // 判断点是否在车身两侧区域（x在[-vehicle_back_length, vehicle_front_length]范围内）
        if (point.x >= -vehicle_back_length && point.x <= vehicle_front_length) {
            points_in_vehicle_side++;
        } else {
            points_in_front_back++;
        }
    }

    int center_point_idx = 0;
    
    // 如果聚类点主要在车身两侧（y轴方向），选择y轴距离短的点为中心点
    if (points_in_vehicle_side >= points_in_front_back) {
        //使用cluster 中最小的std::abs(y)值来赋值
        double min_y_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < cluster.points->points.size(); ++i) {
            const auto &point = cluster.points->points[i];
            double y_distance = std::abs(point.y);
            if (y_distance < min_y_distance) {
                min_y_distance = y_distance;
                center_point_idx = i;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "聚类在车身两侧，选择y轴距离最短的点作为中心点 (y距离: %.3f)", min_y_distance);
    } 
    // 如果聚类点在车前或车后（x轴方向），选择x轴距离短的点为中心点
    else {
        double min_x_distance = std::numeric_limits<double>::max();
        for (size_t i = 0; i < cluster.points->points.size(); ++i) {
            const auto &point = cluster.points->points[i];
            double x_distance = std::abs(point.x);
            if (x_distance < min_x_distance) {
                min_x_distance = x_distance;
                center_point_idx = i;
            }
        }
        RCLCPP_DEBUG(this->get_logger(), "聚类在车前或车后，选择x轴距离最短的点作为中心点 (x距离: %.3f)", min_x_distance);
    }

    // 设置中心点
    cluster.center_x = cluster.points->points[center_point_idx].x;
    cluster.center_y = cluster.points->points[center_point_idx].y;
    cluster.center_z = cluster.points->points[center_point_idx].z;
    // 计算尺寸
    cluster.width = cluster.max_x - cluster.min_x;
    cluster.height = cluster.max_y - cluster.min_y;
    cluster.depth = cluster.max_z - cluster.min_z;
    RCLCPP_DEBUG(this->get_logger(), "聚类信息（基于车辆位置）: 中心(%.3f, %.3f, %.3f), 尺寸(%.3f x %.3f x %.3f),min_z(%.3f), max_z(%.3f),min_x(%.3f), max_x(%.3f),min_y(%.3f), max_y(%.3f), zero_z_point_count(%d),点数: %d", 
                 cluster.center_x, cluster.center_y, cluster.center_z, cluster.width, cluster.height, cluster.depth, cluster.min_z, cluster.max_z, cluster.min_x, cluster.max_x, cluster.min_y, cluster.max_y, cluster.zero_z_point_count, cluster.point_count);
}

void Clustering::publishClusterPointCloud(const std::vector<ClusterInfo> &clusters, const std::string &source_name) {
    if (clusters.empty()) {
        return;
    }

    // 合并所有聚类的点云
    auto combined_cloud = std::make_shared<PointCloud>();
    for (const auto &cluster : clusters) {
        if (cluster.points && !cluster.points->points.empty()) {
            *combined_cloud += *cluster.points;
        }
    }

    if (combined_cloud->points.empty()) {
        return;
    }

    // 转换为ROS2点云消息
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*combined_cloud, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "base_link";

    // 根据源名称选择对应的发布器
    if (source_name == "left" && left_cluster_publisher_) {
        left_cluster_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布左雷达聚类点云数据: %zu 个聚类, %zu 个点", clusters.size(), combined_cloud->points.size());
    } else if (source_name == "right" && right_cluster_publisher_) {
        right_cluster_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布右雷达聚类点云数据: %zu 个聚类, %zu 个点", clusters.size(), combined_cloud->points.size());
    } else if (source_name == "left_tilt" && left_tilt_cluster_publisher_) {
        left_tilt_cluster_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布左倾斜雷达聚类点云数据: %zu 个聚类, %zu 个点", clusters.size(), combined_cloud->points.size());
    } else if (source_name == "right_tilt" && right_tilt_cluster_publisher_) {
        right_tilt_cluster_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布右倾斜雷达聚类点云数据: %zu 个聚类, %zu 个点", clusters.size(), combined_cloud->points.size());
    } else if (source_name == "back" && back_cluster_publisher_) {
        back_cluster_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布后雷达聚类点云数据: %zu 个聚类, %zu 个点", clusters.size(), combined_cloud->points.size());
    }
}
