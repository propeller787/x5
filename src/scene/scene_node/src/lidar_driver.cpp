/**
 * @file lidar_driver.cpp
 * @author likang (likang@freego.com)
 * @brief 接收雷达数据实现
 * @version 0.1
 * @date 2025-10-16
 * 
 * @copyright Copyright (c) 2025
 */

#include "lidar_driver.h"
#include "../../../common/utils/topic.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

LidarDriver::LidarDriver() : rclcpp::Node("lidar_driver") {
    RCLCPP_INFO(this->get_logger(), "初始化雷达驱动器");
    
    // 声明并获取Z方向过滤阈值参数（默认0.3米）
    this->declare_parameter<double>("tilt_z_filter_threshold", 0.27);
    tilt_z_filter_threshold_ = this->get_parameter("tilt_z_filter_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "倾斜雷达Z方向过滤阈值: %.3f 米", tilt_z_filter_threshold_);
    
    // 初始化TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // 初始化ROI区域配置
    initializeROIRegions();
    // 创建订阅器
    RCLCPP_INFO(this->get_logger(), "创建左雷达订阅器，话题: %s", leftLaserTopic);
    left_laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftLaserTopic, rclcpp::SensorDataQoS(), 
        std::bind(&LidarDriver::leftLaserCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "创建右雷达订阅器，话题: %s", rightLaserTopic);
    right_laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightLaserTopic, rclcpp::SensorDataQoS(), 
        std::bind(&LidarDriver::rightLaserCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "创建左倾斜雷达订阅器，话题: %s", leftTiltPointTopice);
    left_tilt_point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        leftTiltPointTopice, rclcpp::SensorDataQoS(), 
        std::bind(&LidarDriver::leftTiltPointCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "创建右倾斜雷达订阅器，话题: %s", rightTiltPointTopice);
    right_tilt_point_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        rightTiltPointTopice, rclcpp::SensorDataQoS(), 
        std::bind(&LidarDriver::rightTiltPointCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "创建后雷达订阅器，话题: %s", backLaserTopic);
    back_laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        backLaserTopic, rclcpp::SensorDataQoS(), 
        std::bind(&LidarDriver::backLaserCallback, this, std::placeholders::_1));
    
    // 创建base_link坐标系转换后的点云发布器
    RCLCPP_INFO(this->get_logger(), "创建左雷达base_link点云发布器，话题: %s", leftLaserBaseLinkTopic);
    left_laser_base_link_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        leftLaserBaseLinkTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建右雷达base_link点云发布器，话题: %s", rightLaserBaseLinkTopic);
    right_laser_base_link_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        rightLaserBaseLinkTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建左倾斜雷达base_link点云发布器，话题: %s", leftTiltBaseLinkTopic);
    left_tilt_base_link_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        leftTiltBaseLinkTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建右倾斜雷达base_link点云发布器，话题: %s", rightTiltBaseLinkTopic);
    right_tilt_base_link_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        rightTiltBaseLinkTopic, 10);
    RCLCPP_INFO(this->get_logger(), "创建后雷达base_link点云发布器，话题: %s", backLaserBaseLinkTopic);
    back_laser_base_link_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        backLaserBaseLinkTopic, 10);
    
    RCLCPP_INFO(this->get_logger(), "雷达驱动器初始化完成");
}

LidarDriver::~LidarDriver() {
    RCLCPP_INFO(this->get_logger(), "雷达驱动器析构完成");
}

void LidarDriver::setPointCloudCallback(std::function<void(const PointCloud::Ptr&, const std::string&)> callback) {
    pointcloud_callback_ = callback;
}


void LidarDriver::leftLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!pointcloud_callback_) {
        return;
    }
    auto pointcloud = laserScanToPointCloud(msg, "laser_frame_left");
    if (pointcloud && !pointcloud->points.empty()) {
        // 先进行坐标转换到base_link
        auto transformed_cloud = transformToBaseLink(pointcloud, "laser_frame_left");
        if (transformed_cloud && !transformed_cloud->points.empty()) {
            // 对所有ROI区域进行过滤
            auto roi_filtered_map = applyAllROIFilters(transformed_cloud);
            
            // 合并所有ROI区域的数据
            auto merged_cloud = std::make_shared<PointCloud>();
            merged_cloud->header = transformed_cloud->header;
            merged_cloud->is_dense = false;
            
            for (const auto& [region_type, roi_cloud] : roi_filtered_map) {
                if (roi_cloud && !roi_cloud->points.empty()) {
                    merged_cloud->points.insert(merged_cloud->points.end(), 
                                              roi_cloud->points.begin(), roi_cloud->points.end());
                }
            }
            
            merged_cloud->width = merged_cloud->points.size();
            merged_cloud->height = 1;
            
            // 通过pointcloud_callback_回调返回合并后的数据
            if (!merged_cloud->points.empty()) {
                pointcloud_callback_(merged_cloud, "left");
            }
            
            publishBaseLinkPointCloud(merged_cloud, "left");
             
        }
    }
}

void LidarDriver::rightLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!pointcloud_callback_) {
        return;
    }
    
    auto pointcloud = laserScanToPointCloud(msg, "laser_frame_right");
    if (pointcloud && !pointcloud->points.empty()) {
        // 先进行坐标转换到base_link
        auto transformed_cloud = transformToBaseLink(pointcloud, "laser_frame_right");
        if (transformed_cloud && !transformed_cloud->points.empty()) {
            // 对所有ROI区域进行过滤
            auto roi_filtered_map = applyAllROIFilters(transformed_cloud);
            
            // 合并所有ROI区域的数据
            auto merged_cloud = std::make_shared<PointCloud>();
            merged_cloud->header = transformed_cloud->header;
            merged_cloud->is_dense = false;
            
            for (const auto& [region_type, roi_cloud] : roi_filtered_map) {
                if (roi_cloud && !roi_cloud->points.empty()) {
                    merged_cloud->points.insert(merged_cloud->points.end(), 
                                              roi_cloud->points.begin(), roi_cloud->points.end());
                }
            }
            
            merged_cloud->width = merged_cloud->points.size();
            merged_cloud->height = 1;
            
            // 通过pointcloud_callback_回调返回合并后的数据
            if (!merged_cloud->points.empty()) {
                pointcloud_callback_(merged_cloud, "right");
            }
            publishBaseLinkPointCloud(merged_cloud, "right");
        }
    }
}

void LidarDriver::leftTiltPointCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!pointcloud_callback_) {
        return;
    }
    
    // 将PointCloud2转换为PCL点云
    auto pointcloud = pointCloud2ToPCL(msg);
    if (pointcloud && !pointcloud->points.empty()) {
        // 获取原始frame_id
        std::string source_frame = msg->header.frame_id.empty() ? "left_tilt_rotate" : msg->header.frame_id;
        
        // 先进行坐标转换到base_link（不转换z轴）
        auto transformed_cloud = transformToBaseLinkWithoutZ(pointcloud, source_frame);
        if (transformed_cloud && !transformed_cloud->points.empty()) {
            // 对Z方向进行过滤，过滤掉z小于阈值的点
            auto z_filtered_cloud = filterByZThreshold(transformed_cloud, tilt_z_filter_threshold_);
            if (!z_filtered_cloud || z_filtered_cloud->points.empty()) {
                return;
            }
            
            // 对所有ROI区域进行过滤
            auto roi_filtered_map = applyAllROIFilters(z_filtered_cloud);
            
            // 合并所有ROI区域的数据
            auto merged_cloud = std::make_shared<PointCloud>();
            merged_cloud->header = z_filtered_cloud->header;
            merged_cloud->is_dense = false;
            
            for (const auto& [region_type, roi_cloud] : roi_filtered_map) {
                if (roi_cloud && !roi_cloud->points.empty()) {
                    merged_cloud->points.insert(merged_cloud->points.end(), 
                                              roi_cloud->points.begin(), roi_cloud->points.end());
                }
            }
            
            merged_cloud->width = merged_cloud->points.size();
            merged_cloud->height = 1;
            
            // 通过pointcloud_callback_回调返回合并后的数据
            if (!merged_cloud->points.empty()) {
                pointcloud_callback_(merged_cloud, "left_tilt");
            }
            publishBaseLinkPointCloud(merged_cloud, "left_tilt");   
        }
    }
}

void LidarDriver::rightTiltPointCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (!pointcloud_callback_) {
        return;
    }
    
    // 将PointCloud2转换为PCL点云
    auto pointcloud = pointCloud2ToPCL(msg);
    if (pointcloud && !pointcloud->points.empty()) {
        // 获取原始frame_id
        std::string source_frame = msg->header.frame_id.empty() ? "right_tilt_rotate" : msg->header.frame_id;
        
        // 先进行坐标转换到base_link（不转换z轴）
        auto transformed_cloud = transformToBaseLinkWithoutZ(pointcloud, source_frame);
        if (transformed_cloud && !transformed_cloud->points.empty()) {
            // 对Z方向进行过滤，过滤掉z小于阈值的点
            auto z_filtered_cloud = filterByZThreshold(transformed_cloud, tilt_z_filter_threshold_);
            if (!z_filtered_cloud || z_filtered_cloud->points.empty()) {
                return;
            }
            
            // 对所有ROI区域进行过滤
            auto roi_filtered_map = applyAllROIFilters(z_filtered_cloud);
            
            // 合并所有ROI区域的数据
            auto merged_cloud = std::make_shared<PointCloud>();
            merged_cloud->header = z_filtered_cloud->header;
            merged_cloud->is_dense = false;
            
            for (const auto& [region_type, roi_cloud] : roi_filtered_map) {
                if (roi_cloud && !roi_cloud->points.empty()) {
                    merged_cloud->points.insert(merged_cloud->points.end(), 
                                              roi_cloud->points.begin(), roi_cloud->points.end());
                }
            }
            
            merged_cloud->width = merged_cloud->points.size();
            merged_cloud->height = 1;
            
            // 通过pointcloud_callback_回调返回合并后的数据
            if (!merged_cloud->points.empty()) {
                pointcloud_callback_(merged_cloud, "right_tilt");
            }
            
            publishBaseLinkPointCloud(merged_cloud, "right_tilt");
        }
    }
}

void LidarDriver::backLaserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!pointcloud_callback_) {
        return;
    }
    
    auto pointcloud = laserScanToPointCloud(msg, "laser_frame_back");
    if (pointcloud && !pointcloud->points.empty()) {
        // 先进行坐标转换到base_link
        auto transformed_cloud = transformToBaseLink(pointcloud, "laser_frame_back");
        if (transformed_cloud && !transformed_cloud->points.empty()) {
            // 对所有ROI区域进行过滤
            auto roi_filtered_map = applyAllROIFilters(transformed_cloud);
            
            // 合并所有ROI区域的数据
            auto merged_cloud = std::make_shared<PointCloud>();
            merged_cloud->header = transformed_cloud->header;
            merged_cloud->is_dense = false;
            
            for (const auto& [region_type, roi_cloud] : roi_filtered_map) {
                if (roi_cloud && !roi_cloud->points.empty()) {
                    merged_cloud->points.insert(merged_cloud->points.end(), 
                                              roi_cloud->points.begin(), roi_cloud->points.end());
                }
            }
            
            merged_cloud->width = merged_cloud->points.size();
            merged_cloud->height = 1;
            
            // 通过pointcloud_callback_回调返回合并后的数据
            if (!merged_cloud->points.empty()) {
                pointcloud_callback_(merged_cloud, "back");
            }
            publishBaseLinkPointCloud(merged_cloud, "back");
        }
    }
}

PointCloud::Ptr LidarDriver::laserScanToPointCloud(const sensor_msgs::msg::LaserScan::SharedPtr& scan_msg, const std::string& frame_id) {
    auto cloud = std::make_shared<PointCloud>();
    
    // 检查数据有效性
    if (scan_msg->ranges.empty()) {
        RCLCPP_WARN(this->get_logger(), "接收到的雷达数据为空");
        return cloud;
    }
    
    // 设置点云头信息
    cloud->header.frame_id = frame_id;
    cloud->header.stamp = pcl_conversions::toPCL(scan_msg->header.stamp);
    cloud->is_dense = false;
    
    // 转换激光扫描数据为点云
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i];
        
        // 过滤无效数据
        if (std::isnan(range) || std::isinf(range) || 
            range < scan_msg->range_min || range > scan_msg->range_max) {
            continue;
        }
        
        // 计算角度
        float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        
        // 计算点坐标
        PointType point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        point.z = 0.0;  // 2D激光雷达，z坐标为0
        
        cloud->points.push_back(point);
    }
    
    cloud->width = cloud->points.size();
    cloud->height = 1;
    
    RCLCPP_DEBUG(this->get_logger(), "转换得到 %zu 个点", cloud->points.size());
    
    return cloud;
}

PointCloud::Ptr LidarDriver::pointCloud2ToPCL(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
    auto cloud = std::make_shared<PointCloud>();
    
    // 检查数据有效性
    if (!cloud_msg || cloud_msg->data.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "接收到的点云数据为空");
        return cloud;
    }
    
    // 从ROS2 PointCloud2消息转换为PCL点云
    pcl::fromROSMsg(*cloud_msg, *cloud);
    
    RCLCPP_DEBUG(this->get_logger(), "转换得到 %zu 个点，frame_id: %s", 
                 cloud->points.size(), cloud->header.frame_id.c_str());
    
    return cloud;
}

PointCloud::Ptr LidarDriver::transformToBaseLink(const PointCloud::Ptr& input_cloud, const std::string& source_frame) {
    if (!input_cloud || input_cloud->points.empty()) {
        return input_cloud;
    }
    
    try {
        // 将PCL点云转换为ROS2点云消息
        sensor_msgs::msg::PointCloud2 input_cloud_msg;
        pcl::toROSMsg(*input_cloud, input_cloud_msg);
        input_cloud_msg.header.stamp = this->now();
        input_cloud_msg.header.frame_id = source_frame;
        
        // 使用TF2进行坐标转换
        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
        tf2::doTransform(input_cloud_msg, transformed_cloud_msg, 
                        tf_buffer_->lookupTransform("base_link", source_frame, 
                                                  tf2::TimePointZero, tf2::durationFromSec(0.1)));
        
        // 转换回PCL点云
        auto transformed_cloud = std::make_shared<PointCloud>();
        pcl::fromROSMsg(transformed_cloud_msg, *transformed_cloud);
        
        RCLCPP_DEBUG(this->get_logger(), "TF2坐标转换完成: %s -> base_link, 点数: %zu", 
                     source_frame.c_str(), transformed_cloud->points.size());
        
        return transformed_cloud;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF2转换失败: %s", ex.what());
        // 如果TF2转换失败，返回原始点云但修改frame_id
        auto fallback_cloud = std::make_shared<PointCloud>(*input_cloud);
        fallback_cloud->header.frame_id = "base_link";
        return fallback_cloud;
    }
}

PointCloud::Ptr LidarDriver::transformToBaseLinkWithoutZ(const PointCloud::Ptr& input_cloud, const std::string& source_frame) {
    if (!input_cloud || input_cloud->points.empty()) {
        return input_cloud;
    }
    
    // 保存原始z值
    std::vector<float> original_z_values;
    original_z_values.reserve(input_cloud->points.size());
    for (const auto& point : input_cloud->points) {
        original_z_values.push_back(point.z);
    }
    
    try {
        // 将PCL点云转换为ROS2点云消息
        sensor_msgs::msg::PointCloud2 input_cloud_msg;
        pcl::toROSMsg(*input_cloud, input_cloud_msg);
        input_cloud_msg.header.stamp = this->now();
        input_cloud_msg.header.frame_id = source_frame;
        
        // 使用TF2进行坐标转换
        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
        tf2::doTransform(input_cloud_msg, transformed_cloud_msg, 
                        tf_buffer_->lookupTransform("base_link", source_frame, 
                                                  tf2::TimePointZero, tf2::durationFromSec(0.1)));
        
        // 转换回PCL点云
        auto transformed_cloud = std::make_shared<PointCloud>();
        pcl::fromROSMsg(transformed_cloud_msg, *transformed_cloud);
        
        // 恢复原始的z值（不转换z轴）
        if (transformed_cloud->points.size() == original_z_values.size()) {
            for (size_t i = 0; i < transformed_cloud->points.size(); ++i) {
                transformed_cloud->points[i].z = original_z_values[i];
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "点云数量不匹配，无法恢复z值: 原始=%zu, 转换后=%zu", 
                       original_z_values.size(), transformed_cloud->points.size());
        }
        
        RCLCPP_DEBUG(this->get_logger(), "TF2坐标转换完成（不转换z轴）: %s -> base_link, 点数: %zu", 
                     source_frame.c_str(), transformed_cloud->points.size());
        
        return transformed_cloud;
        
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "TF2转换失败: %s", ex.what());
        // 如果TF2转换失败，返回原始点云但修改frame_id
        auto fallback_cloud = std::make_shared<PointCloud>(*input_cloud);
        fallback_cloud->header.frame_id = "base_link";
        return fallback_cloud;
    }
}

PointCloud::Ptr LidarDriver::applyROIFilter(const PointCloud::Ptr& input_cloud, const ROIBox& roi_box) {
    if (!input_cloud || input_cloud->points.empty()) {
        return input_cloud;
    }
    
    auto transformed_cloud = std::make_shared<PointCloud>();
    
    // 手动过滤ROI区域
    for (const auto& point : input_cloud->points) {
        if (point.x >= roi_box.min_x && point.x <= roi_box.max_x &&
            point.y >= roi_box.min_y && point.y <= roi_box.max_y &&
            point.z >= roi_box.min_z && point.z <= roi_box.max_z) {
            transformed_cloud->points.push_back(point);
        }
    }
    
    // 设置点云属性
    transformed_cloud->header = input_cloud->header;
    transformed_cloud->width = transformed_cloud->points.size();
    transformed_cloud->height = 1;
    transformed_cloud->is_dense = false;
    
    RCLCPP_DEBUG(this->get_logger(), "ROI过滤: %zu -> %zu 个点", 
                 input_cloud->points.size(), transformed_cloud->points.size());
    
    return transformed_cloud;
}


void LidarDriver::publishBaseLinkPointCloud(const PointCloud::Ptr& transformed_cloud, const std::string& source_name) {
    if (!transformed_cloud || transformed_cloud->points.empty()) {
        return;
    }
    
    // 转换为ROS2点云消息
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*transformed_cloud, cloud_msg);
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = "base_link";
    
    // 根据源名称选择对应的发布器
    if (source_name == "left" && left_laser_base_link_publisher_) {
        left_laser_base_link_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布左雷达base_link点云数据: %zu 个点", transformed_cloud->points.size());
    } else if (source_name == "right" && right_laser_base_link_publisher_) {
        right_laser_base_link_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布右雷达base_link点云数据: %zu 个点", transformed_cloud->points.size());
    } else if (source_name == "left_tilt" && left_tilt_base_link_publisher_) {
        left_tilt_base_link_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布左倾斜雷达base_link点云数据: %zu 个点", transformed_cloud->points.size());
    } else if (source_name == "right_tilt" && right_tilt_base_link_publisher_) {
        right_tilt_base_link_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布右倾斜雷达base_link点云数据: %zu 个点", transformed_cloud->points.size());
    } else if (source_name == "back" && back_laser_base_link_publisher_) {
        back_laser_base_link_publisher_->publish(cloud_msg);
        RCLCPP_DEBUG(this->get_logger(), "发布后雷达base_link点云数据: %zu 个点", transformed_cloud->points.size());
    }
}

void LidarDriver::initializeROIRegions() {
    // 前向区域: [2.0,-0.42],[2.2,0.42],[0.95,-0.42],[0.95,0.42]
    roi_regions_[ROIRegionType::FORWARD] = {
        .min_x = 0.85, .max_x = 3.0,
        .min_y = -0.42, .max_y = 0.42,
        .min_z = -1.0, .max_z = 1.0
    };
    
    // 左侧区域: [0,0.5],[0,0.34],[1.2,0.5],[1.2,0.34]
    roi_regions_[ROIRegionType::LEFT] = {
        .min_x = 0.0, .max_x = 1.2,
        .min_y = 0.30, .max_y = 1.0,
        .min_z = -1.0, .max_z = 1.0
    };
    
    // 右侧区域: [0,-1.0],[0,-0.34],[1.2,-1.0],[1.2,-0.34]
    roi_regions_[ROIRegionType::RIGHT] = {
        .min_x = 0.0, .max_x = 1.2,
        .min_y = -1.0, .max_y = -0.30,
        .min_z = -1.0, .max_z = 1.0
    };
    
    // 后向区域: [-0.24,-0.42],[-0.24,0.42],[-1.24,-0.42],[-1.24,0.42]
    roi_regions_[ROIRegionType::BACKWARD] = {
        .min_x = -2.5, .max_x = -0.20,
        .min_y = -0.42, .max_y = 0.42,
        .min_z = -1.0, .max_z = 1.0
    };
    
    RCLCPP_INFO(this->get_logger(), "ROI区域配置初始化完成");
    RCLCPP_INFO(this->get_logger(), "前向区域: x[%.2f,%.2f], y[%.2f,%.2f]", 
                roi_regions_[ROIRegionType::FORWARD].min_x, roi_regions_[ROIRegionType::FORWARD].max_x,
                roi_regions_[ROIRegionType::FORWARD].min_y, roi_regions_[ROIRegionType::FORWARD].max_y);
    RCLCPP_INFO(this->get_logger(), "左侧区域: x[%.2f,%.2f], y[%.2f,%.2f]", 
                roi_regions_[ROIRegionType::LEFT].min_x, roi_regions_[ROIRegionType::LEFT].max_x,
                roi_regions_[ROIRegionType::LEFT].min_y, roi_regions_[ROIRegionType::LEFT].max_y);
    RCLCPP_INFO(this->get_logger(), "右侧区域: x[%.2f,%.2f], y[%.2f,%.2f]", 
                roi_regions_[ROIRegionType::RIGHT].min_x, roi_regions_[ROIRegionType::RIGHT].max_x,
                roi_regions_[ROIRegionType::RIGHT].min_y, roi_regions_[ROIRegionType::RIGHT].max_y);
    RCLCPP_INFO(this->get_logger(), "后向区域: x[%.2f,%.2f], y[%.2f,%.2f]", 
                roi_regions_[ROIRegionType::BACKWARD].min_x, roi_regions_[ROIRegionType::BACKWARD].max_x,
                roi_regions_[ROIRegionType::BACKWARD].min_y, roi_regions_[ROIRegionType::BACKWARD].max_y);
}

PointCloud::Ptr LidarDriver::applyROIFilterByType(const PointCloud::Ptr& input_cloud, ROIRegionType region_type) {
    if (!input_cloud || input_cloud->points.empty()) {
        return input_cloud;
    }
    
    auto it = roi_regions_.find(region_type);
    if (it == roi_regions_.end()) {
        RCLCPP_WARN(this->get_logger(), "未找到ROI区域类型: %d", static_cast<int>(region_type));
        return input_cloud;
    }
    
    return applyROIFilter(input_cloud, it->second);
}

ROIBox LidarDriver::getROIRegion(ROIRegionType region_type) const {
    auto it = roi_regions_.find(region_type);
    if (it != roi_regions_.end()) {
        return it->second;
    }
    
    // 返回默认ROI区域
    ROIBox default_roi = {0.0, 2.0, -1.0, 1.0, -1.0, 1.0};
    RCLCPP_WARN(this->get_logger(), "使用默认ROI区域: %d", static_cast<int>(region_type));
    return default_roi;
}

std::map<ROIRegionType, PointCloud::Ptr> LidarDriver::applyAllROIFilters(const PointCloud::Ptr& input_cloud) {
    std::map<ROIRegionType, PointCloud::Ptr> roi_filtered_map;
    
    if (!input_cloud || input_cloud->points.empty()) {
        return roi_filtered_map;
    }
    
    // 对所有ROI区域进行过滤
    for (const auto& [region_type, roi_box] : roi_regions_) {
        auto filtered_cloud = applyROIFilter(input_cloud, roi_box);
        if (filtered_cloud && !filtered_cloud->points.empty()) {
            roi_filtered_map[region_type] = filtered_cloud;
            RCLCPP_DEBUG(this->get_logger(), "ROI区域 %d 过滤结果: %zu 个点", 
                         static_cast<int>(region_type), filtered_cloud->points.size());
        }
    }
    
    return roi_filtered_map;
}

std::string LidarDriver::getRegionTypeName(ROIRegionType region_type) const {
    switch (region_type) {
        case ROIRegionType::FORWARD:
            return "forward";
        case ROIRegionType::LEFT:
            return "left";
        case ROIRegionType::RIGHT:
            return "right";
        case ROIRegionType::BACKWARD:
            return "backward";
        default:
            return "unknown";
    }
}

PointCloud::Ptr LidarDriver::filterByZThreshold(const PointCloud::Ptr& input_cloud, double z_threshold) {
    if (!input_cloud || input_cloud->points.empty()) {
        return input_cloud;
    }
    
    auto filtered_cloud = std::make_shared<PointCloud>();
    
    // 过滤掉z小于阈值的点（只保留z >= z_threshold的点）
    for (const auto& point : input_cloud->points) {
        if (point.z <= z_threshold) {
            filtered_cloud->points.push_back(point);
        }
    }
    
    // 设置点云属性
    filtered_cloud->header = input_cloud->header;
    filtered_cloud->width = filtered_cloud->points.size();
    filtered_cloud->height = 1;
    filtered_cloud->is_dense = false;
    
    RCLCPP_DEBUG(this->get_logger(), "Z方向过滤 (阈值: %.3f m): %zu -> %zu 个点", 
                 z_threshold, input_cloud->points.size(), filtered_cloud->points.size());
    
    return filtered_cloud;
}
