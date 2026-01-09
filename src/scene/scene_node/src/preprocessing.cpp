/**
 * @file preprocessing.cpp
 * @author likang (likang@freego.com)
 * @brief 点云数据处理实现
 * @version 2.0
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */

#include "preprocessing.h"
#include <iostream>

Preprocessing::Preprocessing() {
    std::cout << "初始化预处理模块" << std::endl;
}

Preprocessing::~Preprocessing() {
    std::cout << "预处理模块析构" << std::endl;
}

void Preprocessing::setFilteredPointCloudCallback(std::function<void(const PointCloud::Ptr&, const std::string&)> callback) {
    filtered_callback_ = callback;
}

void Preprocessing::processPointCloud(const PointCloud::Ptr& input_cloud, const std::string& source_name) {
    if (!input_cloud || input_cloud->points.empty()) {
        std::cout << "输入点云为空，跳过处理" << std::endl;
        return;
    }
    
    std::cout << "开始处理 " << source_name << " 雷达数据，原始点数: " << input_cloud->points.size() << std::endl;
    
    // 1. 直通滤波 - 距离滤波
    auto filtered_cloud = passThroughFilter(input_cloud);
    std::cout << "直通滤波后点数: " << filtered_cloud->points.size() << std::endl;
    
    if (filtered_cloud->points.empty()) {
        return;
    }
    
    // 2. 体素滤波 - 降采样
    filtered_cloud = voxelFilter(filtered_cloud);
    std::cout << "体素滤波后点数: " << filtered_cloud->points.size() << std::endl;
    
    if (filtered_cloud->points.empty()) {
        return;
    }
    
    // 3. 统计滤波 - 去除离群点
    filtered_cloud = statisticalFilter(filtered_cloud);
    std::cout << "统计滤波后点数: " << filtered_cloud->points.size() << std::endl;
    
    if (filtered_cloud->points.empty()) {
        return;
    }
    
    // 4. 半径滤波 - 去除稀疏点
    filtered_cloud = radiusFilter(filtered_cloud);
    std::cout << "半径滤波后点数: " << filtered_cloud->points.size() << std::endl;
    
    // 触发回调函数
    if (filtered_callback_ && !filtered_cloud->points.empty()) {
        filtered_callback_(filtered_cloud, source_name);
    }
}

PointCloud::Ptr Preprocessing::passThroughFilter(const PointCloud::Ptr& input_cloud) {
    auto filtered_cloud = std::make_shared<PointCloud>();
    
    // 距离滤波
    for (const auto& point : input_cloud->points) {
        double distance = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance >= filter_params_.min_range && distance <= filter_params_.max_range) {
            filtered_cloud->points.push_back(point);
        }
    }
    
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->header = input_cloud->header;
    
    return filtered_cloud;
}

PointCloud::Ptr Preprocessing::voxelFilter(const PointCloud::Ptr& input_cloud) {
    auto filtered_cloud = std::make_shared<PointCloud>();
    
    pcl::VoxelGrid<PointType> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(filter_params_.voxel_leaf_size, 
                            filter_params_.voxel_leaf_size, 
                            filter_params_.voxel_leaf_size);
    voxel_filter.filter(*filtered_cloud);
    
    return filtered_cloud;
}

PointCloud::Ptr Preprocessing::statisticalFilter(const PointCloud::Ptr& input_cloud) {
    auto filtered_cloud = std::make_shared<PointCloud>();
    
    pcl::StatisticalOutlierRemoval<PointType> statistical_filter;
    statistical_filter.setInputCloud(input_cloud);
    statistical_filter.setMeanK(filter_params_.statistical_mean_k);
    statistical_filter.setStddevMulThresh(filter_params_.statistical_std_dev);
    statistical_filter.filter(*filtered_cloud);
    
    return filtered_cloud;
}

PointCloud::Ptr Preprocessing::radiusFilter(const PointCloud::Ptr& input_cloud) {
    auto filtered_cloud = std::make_shared<PointCloud>();
    
    pcl::RadiusOutlierRemoval<PointType> radius_filter;
    radius_filter.setInputCloud(input_cloud);
    radius_filter.setRadiusSearch(filter_params_.radius_search);
    radius_filter.setMinNeighborsInRadius(filter_params_.radius_min_neighbors);
    radius_filter.filter(*filtered_cloud);
    
    return filtered_cloud;
}
