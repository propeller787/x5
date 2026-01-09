#ifndef __LIDAR_OBSTACLE_GROUND_H__
#define __LIDAR_OBSTACLE_GROUND_H__

#include "dbscan.h"
#include "perception_utils.h"
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <utility>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/common/transforms.h>
#include <pcl/pcl_base.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/search/kdtree.h>

#include "interface/msg/object.hpp"
#include "interface/msg/objects.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>

#include "utils/topic.h"
#include <yaml-cpp/yaml.h>


class Lidar_Obstacle_GroundModel : public rclcpp::Node {
public:
    explicit Lidar_Obstacle_GroundModel(std::string name);
    void get_obstacle_pointcloud(PointCloud::Ptr cloud_tranform, PointCloud::Ptr obstacle, const double sort_angle, const Eigen::Matrix4d TransMatrix);
    void scan_callback_right(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void scan_callback_left(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void get_min_cluster_filter(PointCloud::Ptr input, PointCloud::Ptr output_obstacle, PointCloud::Ptr output_ground, double r, int min_points, int mode, 
        const Eigen::Matrix3d R, const Eigen::Vector3d T, PointCloud::Ptr ground_points, bool is_ground_points, bool from_left_lidar_);

    void judge_near_point(PointCloud::Ptr obstacle_all, PointCloud::Ptr ground_all,
    PointCloud::Ptr obstacle_filter, PointCloud::Ptr ground_res, double r, int min_points, int mode, 
    const Eigen::Matrix3d R, const Eigen::Vector3d T);

    void get_base_link_matrix(std::string flie_name, double& Tz, double& Ty, double& Tx, double& Rz, double& Ry, double& Rx, double& sort_angle);
    
    bool check_data(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, float check_range, int min_num);
    void read_from_yaml(std::string flie_name, bool is_left_lidar_, bool is_right_lidar_);
    void get_obstacle(PointCloud::Ptr obstacle_coarse, PointCloud::Ptr ground_coarse,
    PointCloud::Ptr obstacle_res, PointCloud::Ptr ground_res,
    const double sort_angle, const Eigen::Matrix4d TransMatrix, 
    const std::vector<int>& segment_counts_side_ref_, const std::vector<int>& segment_counts_front_ref_,
    const PointT& pcl_start_point_side, const PointT& pcl_end_point_side,
    const PointT& pcl_start_point_front, const PointT& pcl_end_point_front, bool from_left_lidar_);
    void pedal_detect();
    void get_blind_spot_side(PointCloud::Ptr ground_pointcloud, const PointT& pcl_start_point, const PointT& pcl_end_point, PointCloud::Ptr obstacle_cluster);
    void get_blind_spot_front(PointCloud::Ptr ground_pointcloud, const PointT& pcl_start_point, const PointT& pcl_end_point, PointCloud::Ptr obstacle_cluster);
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_right_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_left_;
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pointcloud2_right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pointcloud2_left_pub_;


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transform_pointcloud2_right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transform_pointcloud2_left_pub_;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Logger logger_;

    Eigen::Matrix4d TransMatrix_left;
    Eigen::Matrix4d TransMatrix_right;

    double left_z_ = 0;
    double left_y_ = 0;
    double left_x_ = 0;
    double left_Rz1_ = 0;
    double left_Ry1_ = 0;
    double left_Rx1_ = 0;
    double left_sort_angle_ = 0;
    double left_k_ = 0;
    double left_b_ = 0;
    PointT pcl_start_point_side_left_;
    PointT pcl_end_point_side_left_;
    std::vector<int> segment_counts_side_left_ref_;
    PointT pcl_start_point_front_left_;
    PointT pcl_end_point_front_left_;
    std::vector<int> segment_counts_front_left_ref_;

    double right_z_ = 0;
    double right_y_ = 0;
    double right_x_ = 0;
    double right_Rz1_ = 0;
    double right_Ry1_ = 0;
    double right_Rx1_ = 0;
    double right_sort_angle_ = 0;
    double right_k_ = 0;
    double right_b_ = 0;
    PointT pcl_start_point_side_right_;
    PointT pcl_end_point_side_right_;
    std::vector<int> segment_counts_side_right_ref_;
    PointT pcl_start_point_front_right_;
    PointT pcl_end_point_front_right_;
    std::vector<int> segment_counts_front_right_ref_;

    PointCloud::Ptr cloud_for_pedal_left_ = std::make_shared<PointCloud>();
    PointCloud::Ptr cloud_for_pedal_right_ = std::make_shared<PointCloud>();

    rclcpp::TimerBase::SharedPtr timer_pedal_detect_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pedal_limiter_pub_;
    int last_alert_value_ = 0;

    int pedal_limit_count_ = 0;
    int pedal_limit_count2_ = 0;
    int pedal_normal_count_ = 0;


};

#endif // __LIDAR_OBSTACLE_GROUND_H__
