#ifndef __PODAO_DETECT_H__
#define __PODAO_DETECT_H__

#include "dbscan.h"
#include "perception_utils.h"
#include <laser_geometry/laser_geometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/imu.hpp>

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
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include <podao_detect/fit_line.h>


enum Status {
    Flat = 100,
    Up = 101,
    Down = 102,
    FlatToUp = 103,
    FlatToDown = 104
};

class Podao_Detect : public rclcpp::Node {
public:
    explicit Podao_Detect(std::string name);
    void get_podao_line(PointCloud::Ptr cloud, PointCloud::Ptr podao_line, std::vector<PointCloud::Ptr> &pdao_line_vector);
    void scan_callback_right(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void scan_callback_left(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    // void scan_callback_right_Hori(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    // void scan_callback_left_Hori(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    void get_min_cluster_filter(PointCloud::Ptr input, std::vector<PointCloud::Ptr>& pdao_line_vector, double r, int min_points, int mode);

    void get_base_link_matrix(std::string flie_name, double& Tz, double& Ty, double& Tx, double& Rz, double& Ry, double& Rx, double& sort_angle);
    
    bool check_data(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, float check_range, int min_num);
    void read_from_yaml(std::string flie_name);

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    // void front_obstacle_Callback();

    void listener_scene_mode_callback(const std_msgs::msg::String::SharedPtr msg);
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_right_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_left_;
    // 创建 水平LaserScan 消息订阅者
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_right_Hori_;
    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_left_Hori_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr scene_mode_sub_;

    // 发布者
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr podao_pointcloud2_left_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr podao_pointcloud2_right_pub_;

    


    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transform_pointcloud2_right_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transform_pointcloud2_left_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr status_publisher_;



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

    double right_z_ = 0;
    double right_y_ = 0;
    double right_x_ = 0;
    double right_Rz1_ = 0;
    double right_Ry1_ = 0;
    double right_Rx1_ = 0;
    double right_sort_angle_ = 0;

    double pitch_base_ = 2; // 基准点的 pitch 角度
    int down_count_ = 0; // 用于计数的变量
    int flat_count_ = 0; // 用于计数的变量
    int up_count_ = 0; // 用于计数的变量

    Status currentStatus_ = Flat;
    Status historyStatus_ = Flat;

    Status historyStatus_lidar_left_ = Flat;
    Status historyStatus_lidar_right_ = Flat;

    std::vector<PointCloud::Ptr> podao_line_vector_left_;
    std::vector<PointCloud::Ptr> podao_line_vector_right_;

    int futureStatus_right_flat_count_ = 0;
    int futureStatus_right_down_count_ = 0;
    int futureStatus_right_up_count_ = 0;

    int futureStatus_left_up_count_ = 0;
    int futureStatus_left_down_count_ = 0;
    int futureStatus_left_flat_count_ = 0;

    int flattodown2flat_count_ = 0; // 用于计数的变量
    int flattoup2flat_count_ = 0; // 用于计数的变量
    int down2flat_count_ = 0;


    // std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    // std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // tf2::Vector3 translation_left_hori;
    // tf2::Matrix3x3 rotation_matrix_left_hori;
    // float left_lidar_hori_x = 0;
    // float left_lidar_hori_y = 0;

    // tf2::Vector3 translation_right_hori;
    // tf2::Matrix3x3 rotation_matrix_right_hori;
    // float right_lidar_hori_x = 0;
    // float right_lidar_hori_y = 0;


    // PointCloud::Ptr cloud_all_left_hori = std::make_shared<PointCloud>();
    // PointCloud::Ptr cloud_all_right_hori = std::make_shared<PointCloud>();

    // bool is_front_obstacle_ = false;
    // rclcpp::TimerBase::SharedPtr timer_front_obstacle_;


    bool is_in_narrow_mode_ = false;

};

#endif // PODAO_DETECT
