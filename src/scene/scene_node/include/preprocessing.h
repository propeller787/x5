/**
 * @file preprocessing.h
 * @author likang (likang@freego.com)
 * @brief 点云数据处理
 * 包括点云数据滤波、点云数据分割、点云数据配准等
 * @version 2.0
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <memory>
#include <functional>

// 定义点云类型
using PointType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointType>;

class Preprocessing {
public:
    Preprocessing();
    ~Preprocessing();
    
    // 设置滤波后的点云回调函数
    void setFilteredPointCloudCallback(std::function<void(const PointCloud::Ptr&, const std::string&)> callback);
    
    // 处理点云数据
    void processPointCloud(const PointCloud::Ptr& input_cloud, const std::string& source_name);

private:
    // 体素滤波 - 降采样
    PointCloud::Ptr voxelFilter(const PointCloud::Ptr& input_cloud);
    
    // 统计滤波 - 去除离群点
    PointCloud::Ptr statisticalFilter(const PointCloud::Ptr& input_cloud);
    
    // 半径滤波 - 去除稀疏点
    PointCloud::Ptr radiusFilter(const PointCloud::Ptr& input_cloud);
    
    // 直通滤波 - 距离滤波
    PointCloud::Ptr passThroughFilter(const PointCloud::Ptr& input_cloud);
    
    // 滤波后的点云回调函数
    std::function<void(const PointCloud::Ptr&, const std::string&)> filtered_callback_;
    
    // 滤波参数
    struct FilterParams {
        double voxel_leaf_size = 0.05;        // 体素滤波叶子大小 (m)
        int statistical_mean_k = 20;          // 统计滤波邻近点数
        double statistical_std_dev = 1.0;     // 统计滤波标准差倍数
        double radius_search = 0.1;           // 半径滤波搜索半径 (m)
        int radius_min_neighbors = 3;         // 半径滤波最小邻近点数
        double min_range = 0.1;               // 最小距离 (m)
        double max_range = 30.0;              // 最大距离 (m)
    } filter_params_;
};