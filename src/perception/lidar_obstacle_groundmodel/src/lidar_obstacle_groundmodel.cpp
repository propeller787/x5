#include "lidar_obstacle_groundmodel.h"
#include <iostream>
#include <queue>
#include <vector>

bool customCompare(PointT a, PointT b, Eigen::Matrix3d RotateMatrix, Eigen::Vector3d TransVector)
{
    PointT a_trans;

    Eigen::Vector3d point_eigen_a(a.x, a.y, a.z);
    point_eigen_a = point_eigen_a - TransVector;
    point_eigen_a = RotateMatrix * point_eigen_a;

    a_trans.x = point_eigen_a[0];
    a_trans.y = point_eigen_a[1];
    a_trans.z = point_eigen_a[2];

    float a_angle = atan2(a_trans.y, a_trans.x);
    if (a_angle < 0) {
        a_angle += 2 * M_PI;
    }
    a_angle -= M_PI;
    if (a_angle < 0) {
        a_angle += 2 * M_PI;
    }

    PointT b_trans;

    Eigen::Vector3d point_eigen_b(b.x, b.y, b.z);
    point_eigen_b = point_eigen_b - TransVector;
    point_eigen_b = RotateMatrix * point_eigen_b;

    b_trans.x = point_eigen_b[0];
    b_trans.y = point_eigen_b[1];
    b_trans.z = point_eigen_b[2];

    float b_angle = atan2(b_trans.y, b_trans.x);
    if (b_angle < 0) {
        b_angle += 2 * M_PI;
    }
    b_angle -= M_PI;
    if (b_angle < 0) {
        b_angle += 2 * M_PI;
    }

    return a_angle < b_angle;
}

// 将两个 pcl::PointXYZ 转换为 Eigen::Vector3d
Eigen::Vector3d toEigen(const pcl::PointXYZ& pt) {
    return Eigen::Vector3d(pt.x, pt.y, pt.z);
}

// 统计每段的点数量
void countPointsInSegments(const pcl::PointXYZ& start_pt,
                           const pcl::PointXYZ& end_pt,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, std::vector<int>& segment_counts) {
    Eigen::Vector3d start = toEigen(start_pt);
    Eigen::Vector3d end = toEigen(end_pt);
    Eigen::Vector3d direction = end - start;
    double length = direction.norm();
    Eigen::Vector3d unit_dir = direction.normalized();
    double segment_length = length / segment_counts.size();

    // std::cout << "Start Point: " << start.transpose() << std::endl;
    // std::cout << "End Point: " << end.transpose() << std::endl;
    // std::cout << "Direction: " << direction.transpose() << std::endl;
    // std::cout << "Unit Direction: " << unit_dir.transpose() << std::endl;

    for (const auto& p : cloud->points) {
        if (abs(p.z > 0.1)) 
        {
            continue;  // 忽略高于1.5m的点
        } 

        Eigen::Vector3d vec = toEigen(p) - start;
        double proj_len = vec.dot(unit_dir);  // 投影长度
        // 判断是否在线段投影范围内
        if (proj_len < 0 || proj_len > length)
            continue;

        // 计算点到线段的垂直距离
        Eigen::Vector3d nearest_point = start + proj_len * unit_dir;
        double distance_to_line = (toEigen(p) - nearest_point).norm();

        if (distance_to_line > 0.1) {
            continue; // 忽略距离过远的点
        }


        int index = static_cast<int>(proj_len / segment_length);
        if (index >= segment_counts.size()) index = segment_counts.size()-1;
        segment_counts[index]++;
    }

    // // 输出结果
    // for (int i = 0; i < segment_counts.size(); ++i) {
    //     std::cout << "Segment " << i << ": " << segment_counts[i] << " points" << std::endl;
    // }
}

void get_max_cluster(PointCloud::Ptr input, PointCloud::Ptr output_obstacle, double r, int min_points, int mode)
{
    
    std::vector<pcl::PointIndices> cluster_indices;
    int max_points = 1000; // 3

    if (input->size() <= 0) {
        return;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input); // 创建点云索引向量，用于存储实际的点云信息
    if (mode == 1) {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(r); // 设置近邻搜索的搜索半径为0.03m
        ec.setMinClusterSize(min_points); // 设置一个聚类需要的最少点数目为15
        ec.setMaxClusterSize(max_points); // 设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod(tree); // 设置点云的搜索机制
        ec.setInputCloud(input); //
        ec.extract(cluster_indices); // 从点云中提取聚类，并将点云索引保存在cluster_indices中/**/
    } else if (mode == 2) {
        DBSCANSimpleCluster<pcl::PointXYZ> ec;
        ec.setCorePointMinPts(1); // 3
        ec.setClusterTolerance(r);
        ec.setMinClusterSize(min_points);
        ec.setMaxClusterSize(max_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input);
        ec.extract(cluster_indices);
    }

    // 选取点数最多的一类
    int max_size = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cloud_cluster(new PointCloud);

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster->points.push_back(input->points[*pit]);
        }

        // if (cloud_cluster->points.size() > max_size) {
        //     max_size = cloud_cluster->points.size();
            *output_obstacle += *cloud_cluster;  // 更新输出点云
        // }
    }
}

std::vector<PointCloud::Ptr> get_clusters(PointCloud::Ptr input, double r, int min_points, int mode)
{
    std::vector<PointCloud::Ptr> cluster_vec;
    std::vector<pcl::PointIndices> cluster_indices;
    int max_points = 1000; // 3

    if (input->size() <= 0) {
        return cluster_vec;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input); // 创建点云索引向量，用于存储实际的点云信息
    if (mode == 1) {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(r); // 设置近邻搜索的搜索半径为0.03m
        ec.setMinClusterSize(min_points); // 设置一个聚类需要的最少点数目为15
        ec.setMaxClusterSize(max_points); // 设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod(tree); // 设置点云的搜索机制
        ec.setInputCloud(input); //
        ec.extract(cluster_indices); // 从点云中提取聚类，并将点云索引保存在cluster_indices中/**/
    } else if (mode == 2) {
        DBSCANSimpleCluster<pcl::PointXYZ> ec;
        ec.setCorePointMinPts(1); // 3
        ec.setClusterTolerance(r);
        ec.setMinClusterSize(min_points);
        ec.setMaxClusterSize(max_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input);
        ec.extract(cluster_indices);
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            cloud_cluster->points.push_back(input->points[*pit]);
        }

        cluster_vec.push_back(cloud_cluster);
    }

    return cluster_vec;
}

// 统计每段的点数量
void countPointsInSegments_All(const pcl::PointXYZ& start_pt,
                           const pcl::PointXYZ& end_pt,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           PointCloud::Ptr output_all) {
    Eigen::Vector3d start = toEigen(start_pt);
    Eigen::Vector3d end = toEigen(end_pt);
    Eigen::Vector3d direction = end - start;
    double length = direction.norm();
    Eigen::Vector3d unit_dir = direction.normalized();
    double segment_length = length ;


    for (const auto& p : cloud->points) {
        if (abs(p.z > 0.1)) 
        {
            continue;  // 忽略高于1.5m的点
        } 

        Eigen::Vector3d vec = toEigen(p) - start;
        double proj_len = vec.dot(unit_dir);  // 投影长度
        // 判断是否在线段投影范围内
        if (proj_len < 0 || proj_len > length)
            continue;

        // 计算点到线段的垂直距离
        Eigen::Vector3d nearest_point = start + proj_len * unit_dir;
        double distance_to_line = (toEigen(p) - nearest_point).norm();

        if (distance_to_line > 0.1) {
            continue; // 忽略距离过远的点
        }

        output_all->points.push_back(p);

    }

    output_all->width = output_all->points.size();
    output_all->height = 1;
    output_all->is_dense = false;
}


Lidar_Obstacle_GroundModel::Lidar_Obstacle_GroundModel(std::string name)
    : Node(name)
    , logger_(rclcpp::get_logger(name))
{

    // tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    std::string path = getProcessPath();
    RCLCPP_INFO(this->get_logger(), "the exe path is %s", path.c_str());
    // std::string ip = getLocalIPAddress();
    // RCLCPP_INFO(this->get_logger(), "the ip is %s", ip.c_str());
    std::string interface_name = "eth0";  // 替换为你的网卡名，如 enp3s0, wlan0
    std::string mac = get_mac_address(interface_name);
    RCLCPP_INFO(this->get_logger(), "The MAC address of %s is: %s", interface_name.c_str(), mac.c_str());
    if (mac == "3A:C4:F0:04:E8:5D") {
        get_base_link_matrix(path + "../../../../src/calibration_files/01/left_tilt_lidar.txt",
            left_z_, left_y_, left_x_, left_Rz1_, left_Ry1_, left_Rx1_, left_sort_angle_);
        // todo read_from_yaml 不传参
        read_from_yaml(path + "../../../../src/calibration_files/01/left_lidar_groung_model.yaml",
            true, false);

        get_base_link_matrix(path + "../../../../src/calibration_files/01/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);
        read_from_yaml(path + "../../../../src/calibration_files/01/right_lidar_ground_model.yaml",
            false, true);
    
    } 
    else if (mac == "66:FA:C4:30:2A:24") {
        get_base_link_matrix(path + "../../../../src/calibration_files/02/left_tilt_lidar.txt",
            left_z_, left_y_, left_x_, left_Rz1_, left_Ry1_, left_Rx1_, left_sort_angle_);
        read_from_yaml(path + "../../../../src/calibration_files/02/left_lidar_groung_model.yaml",
            true, false);

        get_base_link_matrix(path + "../../../../src/calibration_files/02/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);
        read_from_yaml(path + "../../../../src/calibration_files/02/right_lidar_ground_model.yaml",
            false, true);
    }
    else if (mac == "46:F9:C0:AD:42:4C") {
        get_base_link_matrix(path + "../../../../src/calibration_files/03/left_tilt_lidar.txt",
            left_z_, left_y_, left_x_, left_Rz1_, left_Ry1_, left_Rx1_, left_sort_angle_);
        read_from_yaml(path + "../../../../src/calibration_files/03/left_lidar_groung_model.yaml",
            true, false);

        get_base_link_matrix(path + "../../../../src/calibration_files/03/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);
        read_from_yaml(path + "../../../../src/calibration_files/03/right_lidar_ground_model.yaml",
            false, true);
    }
    else if (mac == "96:AF:3E:5F:A9:B8") {
        get_base_link_matrix(path + "../../../../src/calibration_files/04/left_tilt_lidar.txt",
            left_z_, left_y_, left_x_, left_Rz1_, left_Ry1_, left_Rx1_, left_sort_angle_);
        read_from_yaml(path + "../../../../src/calibration_files/04/left_lidar_groung_model.yaml",
            true, false);

        get_base_link_matrix(path + "../../../../src/calibration_files/04/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);
        read_from_yaml(path + "../../../../src/calibration_files/04/right_lidar_ground_model.yaml",
            false, true);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "the ip is not correct");
        return;
    }

    // publisher_
    transform_pointcloud2_right_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transform_pointcloud2_right", 1);
    transform_pointcloud2_left_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/transform_pointcloud2_left", 1);
    
    obstacle_pointcloud2_right_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(rightTiltPointTopice, 1);
    obstacle_pointcloud2_left_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(leftTiltPointTopice, 1);
    
    pedal_limiter_pub_ = this->create_publisher<std_msgs::msg::Int32>("/perception/pedal/limiter/status", 2);
    // 创建 LaserScan 消息订阅者

    scan_subscriber_left_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftTiltLaserTopic, 10,
        std::bind(&Lidar_Obstacle_GroundModel::scan_callback_left, this, std::placeholders::_1));
    scan_subscriber_right_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightTiltLaserTopic, 10,
        std::bind(&Lidar_Obstacle_GroundModel::scan_callback_right, this, std::placeholders::_1));

    timer_pedal_detect_sub_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&Lidar_Obstacle_GroundModel::pedal_detect, this));
}

void Lidar_Obstacle_GroundModel::scan_callback_left(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{

    if (left_z_ == 0) {
        RCLCPP_WARN(this->get_logger(), "failed, the calibration_file is not exist");
        return;
    }
    Eigen::Matrix3d RotateMatrix;
    RotateMatrix = Eigen::AngleAxisd(left_Rz1_, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(left_Ry1_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(left_Rx1_, Eigen::Vector3d::UnitX());
    Eigen::Vector3d TransVector;

    TransVector << 0, 0, left_z_;

    TransMatrix_left.block(0, 0, 3, 3) = RotateMatrix;
    TransMatrix_left.block(0, 3, 3, 1) = TransVector;

    float check_range = 0.25;
    bool no_noise = check_data(scan_msg, check_range, 10);
    // 转换后的点云，debug 用
    PointCloud::Ptr cloud(new PointCloud);
    // y > 0 
    std::vector<float> laser_points_positive;   // diff 
    std::vector<PointT> points_positive;        // point
    // y < 0 
    std::vector<float> laser_points_negative;  // diff 
    std::vector<PointT> points_negative;       // point
    // all
    std::vector<float> laser_points;
    std::vector<PointT> points_all;  

    float angle = scan_msg->angle_min; // 当前角度（从左到右扫描
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i]; // 获取当前点的距离
        PointT point;
        // 如果距离有效（大于最小距离且小于最大距离）
        if (range >= scan_msg->range_min && range <= scan_msg->range_max) {
            if (!no_noise && range < check_range) {
                angle += scan_msg->angle_increment;
                continue;
            }
            // 转换为笛卡尔坐标（x, y）
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0;

            PointT p_base;
            p_base.x = RotateMatrix(0, 0) * point.x + RotateMatrix(0, 1) * point.y + RotateMatrix(0, 2) * point.z + TransVector(0);
            p_base.y = RotateMatrix(1, 0) * point.x + RotateMatrix(1, 1) * point.y + RotateMatrix(1, 2) * point.z + TransVector(1);
            p_base.z = RotateMatrix(2, 0) * point.x + RotateMatrix(2, 1) * point.y + RotateMatrix(2, 2) * point.z + TransVector(2);

            if (p_base.z < 1.5) {
                cloud->points.push_back(p_base);
            }

            float ref_range = 0;
            float range_diff = 0;
            if (angle>0 && angle < M_PI/2) {
                ref_range = left_b_ / (sin(angle) - left_k_ * cos(angle));
                range_diff = range - ref_range;
                laser_points_positive.push_back(range_diff);
                points_positive.push_back(p_base);
                
            } 
            else if(angle > 3* M_PI/2 && angle <=  2*M_PI) {
                ref_range = left_b_ / (sin(angle) - left_k_ * cos(angle));
                range_diff = range - ref_range;
                laser_points_negative.push_back(range_diff);
                points_negative.push_back(p_base);
            }
        }
        // 更新角度
        angle += scan_msg->angle_increment;
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    *cloud_for_pedal_left_ = *cloud;

    sensor_msgs::msg::PointCloud2 ros_msgs_convert;
    PCL2ROSMSg(cloud, ros_msgs_convert);
    ros_msgs_convert.header = scan_msg->header;
    ros_msgs_convert.header.frame_id = "left_tilt_rotate";
    transform_pointcloud2_left_pub_->publish(ros_msgs_convert);

    std::reverse(laser_points_positive.begin(), laser_points_positive.end());
    std::reverse(laser_points_negative.begin(), laser_points_negative.end());
    laser_points.insert(laser_points.end(), laser_points_positive.begin(), laser_points_positive.end());
    laser_points.insert(laser_points.end(), laser_points_negative.begin(), laser_points_negative.end());

    std::reverse(points_positive.begin(), points_positive.end());
    std::reverse(points_negative.begin(), points_negative.end());
    points_all.insert(points_all.end(), points_positive.begin(), points_positive.end());
    points_all.insert(points_all.end(), points_negative.begin(), points_negative.end());

    // 计算障碍物点云
    std::vector<int> laser_points_flag(laser_points.size(), 0);
    for (size_t i = 0; i < laser_points.size(); i++) {
        if (laser_points[i] > 0.15) {
            laser_points_flag[i] = 1;
        } 
        else if (laser_points[i] < -0.15) {
            laser_points_flag[i] = -1;
        }
    }
    


    PointCloud::Ptr obstacle_coarse(new PointCloud);
    PointCloud::Ptr ground_coarse(new PointCloud);

    for (size_t i = 0; i < laser_points_flag.size(); i++) {
        if (abs(laser_points_flag[i]) == 1) {
            PointT p_base;
            p_base = points_all[i];
            obstacle_coarse->points.push_back(p_base);
        }
        else{
            PointT p_base;
            p_base = points_all[i];
            ground_coarse->points.push_back(p_base);
        }
    }

    PointCloud::Ptr obstacle_res(new PointCloud);
    PointCloud::Ptr ground_res(new PointCloud);
    get_obstacle(obstacle_coarse, ground_coarse,
    obstacle_res, ground_res, left_sort_angle_, TransMatrix_left, 
    segment_counts_side_left_ref_, segment_counts_front_left_ref_,
    pcl_start_point_side_left_, pcl_end_point_side_left_, pcl_start_point_front_left_, pcl_end_point_front_left_, true);

    // 发布障碍物点云
    sensor_msgs::msg::PointCloud2 ros_msgs_obstacle;
    PCL2ROSMSg(obstacle_res, ros_msgs_obstacle);
    ros_msgs_obstacle.header = scan_msg->header;
    ros_msgs_obstacle.header.frame_id = "left_tilt_rotate";
    obstacle_pointcloud2_left_pub_->publish(ros_msgs_obstacle);
}

void Lidar_Obstacle_GroundModel::scan_callback_right(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{

    if (right_z_ == 0) {
        RCLCPP_ERROR(this->get_logger(), "failed, the calibration_file is not exist");
        return;
    }
    Eigen::Matrix3d RotateMatrix;
    RotateMatrix = Eigen::AngleAxisd(right_Rz1_, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(right_Ry1_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(right_Rx1_, Eigen::Vector3d::UnitX());
    Eigen::Vector3d TransVector;
    TransVector << 0, 0, right_z_;

    TransMatrix_right.block(0, 0, 3, 3) = RotateMatrix;
    TransMatrix_right.block(0, 3, 3, 1) = TransVector;

    float check_range = 0.25;
    bool no_noise = check_data(scan_msg, check_range, 10);
    // 转换后的点云，debug 用
    PointCloud::Ptr cloud(new PointCloud);
    // y > 0 
    std::vector<float> laser_points_positive;   // diff 
    std::vector<PointT> points_positive;        // point
    // y < 0 
    std::vector<float> laser_points_negative;  // diff 
    std::vector<PointT> points_negative;       // point
    // all
    std::vector<float> laser_points;
    std::vector<PointT> points_all;  

    float angle = scan_msg->angle_min; // 当前角度（从左到右扫描
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i]; // 获取当前点的距离
        PointT point;
        // 如果距离有效（大于最小距离且小于最大距离）
        if (range >= scan_msg->range_min && range <= scan_msg->range_max) {
            // 如果存在噪点，消除距离太小值
            if (!no_noise && range < check_range) {
                angle += scan_msg->angle_increment;
                continue;
            }
            // 转换为笛卡尔坐标（x, y）
            point.x = range * std::cos(angle);
            point.y = range * std::sin(angle);
            point.z = 0;

            PointT p_base;
            p_base.x = RotateMatrix(0, 0) * point.x + RotateMatrix(0, 1) * point.y + RotateMatrix(0, 2) * point.z + TransVector(0);
            p_base.y = RotateMatrix(1, 0) * point.x + RotateMatrix(1, 1) * point.y + RotateMatrix(1, 2) * point.z + TransVector(1);
            p_base.z = RotateMatrix(2, 0) * point.x + RotateMatrix(2, 1) * point.y + RotateMatrix(2, 2) * point.z + TransVector(2);

            if (p_base.z < 1.5) {
                cloud->points.push_back(p_base);
            }

            float ref_range = 0;
            float range_diff = 0;
            if (angle>0 && angle < M_PI/2) {
                ref_range = right_b_ / (sin(angle) - right_k_ * cos(angle));
                range_diff = range - ref_range;
                laser_points_positive.push_back(range_diff);
                points_positive.push_back(p_base);
                
            } 
            else if(angle > 3* M_PI/2 && angle <=  2*M_PI) {
                ref_range = right_b_ / (sin(angle) - right_k_ * cos(angle));
                range_diff = range - ref_range;
                laser_points_negative.push_back(range_diff);
                points_negative.push_back(p_base);
            }
        }
        // 更新角度
        angle += scan_msg->angle_increment;
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    *cloud_for_pedal_right_ = *cloud;

    sensor_msgs::msg::PointCloud2 ros_msgs1;
    PCL2ROSMSg(cloud, ros_msgs1);
    ros_msgs1.header = scan_msg->header;
    ros_msgs1.header.frame_id = "right_tilt_rotate";
    transform_pointcloud2_right_pub_->publish(ros_msgs1);

    std::reverse(laser_points_positive.begin(), laser_points_positive.end());
    std::reverse(laser_points_negative.begin(), laser_points_negative.end());
    laser_points.insert(laser_points.end(), laser_points_positive.begin(), laser_points_positive.end());
    laser_points.insert(laser_points.end(), laser_points_negative.begin(), laser_points_negative.end());

    std::reverse(points_positive.begin(), points_positive.end());
    std::reverse(points_negative.begin(), points_negative.end());
    points_all.insert(points_all.end(), points_positive.begin(), points_positive.end());
    points_all.insert(points_all.end(), points_negative.begin(), points_negative.end());

    // 计算障碍物点云
    std::vector<int> laser_points_flag(laser_points.size(), 0);
    for (size_t i = 0; i < laser_points.size(); i++) {
        if (laser_points[i] > 0.15) {
            laser_points_flag[i] = 1;
        } 
        else if (laser_points[i] < -0.15) {
            laser_points_flag[i] = -1;
        }
    }

    PointCloud::Ptr obstacle_coarse(new PointCloud);
    PointCloud::Ptr ground_coarse(new PointCloud);

    for (size_t i = 0; i < laser_points_flag.size(); i++) {
        if (abs(laser_points_flag[i]) == 1) {
            PointT p_base;
            p_base = points_all[i];
            obstacle_coarse->points.push_back(p_base);
        }
        else{
            PointT p_base;
            p_base = points_all[i];
            ground_coarse->points.push_back(p_base);
        }
    }

    PointCloud::Ptr obstacle_res(new PointCloud);
    PointCloud::Ptr ground_res(new PointCloud);
    get_obstacle(obstacle_coarse, ground_coarse,
    obstacle_res, ground_res, right_sort_angle_, TransMatrix_right, 
    segment_counts_side_right_ref_, segment_counts_front_right_ref_,
    pcl_start_point_side_right_, pcl_end_point_side_right_, pcl_start_point_front_right_, pcl_end_point_front_right_, false);

    // 发布障碍物点云
    sensor_msgs::msg::PointCloud2 ros_msgs_obstacle;
    PCL2ROSMSg(obstacle_res, ros_msgs_obstacle);
    ros_msgs_obstacle.header = scan_msg->header;
    ros_msgs_obstacle.header.frame_id = "right_tilt_rotate";
    obstacle_pointcloud2_right_pub_->publish(ros_msgs_obstacle);
    
}


void Lidar_Obstacle_GroundModel::get_min_cluster_filter(PointCloud::Ptr input, PointCloud::Ptr output_obstacle, PointCloud::Ptr output_ground, double r, int min_points, int mode, 
    const Eigen::Matrix3d R, const Eigen::Vector3d T, PointCloud::Ptr ground_points, bool is_ground_points=false, bool from_left_lidar_=false)
{
    
    std::vector<pcl::PointIndices> cluster_indices;
    int max_points = 1000; // 3

    if (input->size() <= 0) {
        return;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(input); // 创建点云索引向量，用于存储实际的点云信息
    if (mode == 1) {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(r); // 设置近邻搜索的搜索半径为0.03m
        ec.setMinClusterSize(min_points); // 设置一个聚类需要的最少点数目为15
        ec.setMaxClusterSize(max_points); // 设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod(tree); // 设置点云的搜索机制
        ec.setInputCloud(input); //
        ec.extract(cluster_indices); // 从点云中提取聚类，并将点云索引保存在cluster_indices中/**/
    } else if (mode == 2) {
        DBSCANSimpleCluster<pcl::PointXYZ> ec;
        ec.setCorePointMinPts(1); // 3
        ec.setClusterTolerance(r);
        ec.setMinClusterSize(min_points);
        ec.setMaxClusterSize(max_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(input);
        ec.extract(cluster_indices);
    }

    if (is_ground_points && cluster_indices.size() > 0) {
        std::sort(cluster_indices.begin(), cluster_indices.end(), [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
            return a.indices.size() > b.indices.size();
        });
        pcl::PointIndices most_points_indices = cluster_indices.front();
        PointCloud::Ptr cloud_cluster(new PointCloud);
        for (std::vector<int>::const_iterator pit = most_points_indices.indices.begin(); pit != most_points_indices.indices.end(); pit++) {
            cloud_cluster->points.push_back(input->points[*pit]);
        }
        *output_ground += *cloud_cluster;
        cluster_indices.erase(cluster_indices.begin());
        
    }

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        // 只有一个点的情况
        if (it->indices.size() <= 3) {
            if (input->points[*(it->indices.begin())].z < -0.1 || input->points[*(it->indices.begin())].z > 0.1) {
                cloud_cluster->points.push_back(input->points[*(it->indices.begin())]);
                *output_obstacle += *cloud_cluster;
            }
            else {
                cloud_cluster->points.push_back(input->points[*(it->indices.begin())]);
                *output_ground += *cloud_cluster;
            }
        }
        else {
            float min_z, max_z, min_x, max_x, min_y, max_y, max_slope;
            PointT min_z_point, max_z_point;
            min_z = input->points[*(it->indices.begin())].z;
            max_z = input->points[*(it->indices.begin())].z;

            min_x = input->points[*(it->indices.begin())].x;
            max_x = input->points[*(it->indices.begin())].x;

            min_y = input->points[*(it->indices.begin())].y;
            max_y = input->points[*(it->indices.begin())].y;

            min_z_point = input->points[*(it->indices.begin())];
            max_z_point = input->points[*(it->indices.begin())];

            max_slope = 0.0;
            
            int point_count = 0;
            int point_count_neg = 0;
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
                cloud_cluster->points.push_back(input->points[*pit]);
                min_z = std::min(min_z, input->points[*pit].z);
                max_z = std::max(max_z, input->points[*pit].z);
                if (min_z == input->points[*pit].z) {
                    min_z_point = input->points[*pit];
                }

                if (max_z == input->points[*pit].z) {
                    max_z_point = input->points[*pit];
                }

                min_x = std::min(min_x, input->points[*pit].x);
                max_x = std::max(max_x, input->points[*pit].x);

                min_y = std::min(min_y, input->points[*pit].y);
                max_y = std::max(max_y, input->points[*pit].y);

                max_slope = std::max(max_slope, static_cast<float>(std::abs(input->points[*pit].z / (input->points[*pit].x))));

                if (input->points[*pit].x > 0. && input->points[*pit].x < 0.3) {
                    point_count++;
                    if (input->points[*pit].z < -0.1) {
                        point_count_neg++;
                    }
                }

            }

            float det_z = max_z - min_z;
            float det_x = max_x - min_x;
            float det_y = max_y - min_y;
            float slope_x = 0;
            float slope_y = 0;
            float slope_z = 0;
            if (std::abs(max_z_point.x) > std::abs(min_z_point.x)) {
                slope_x = max_z_point.x;
                slope_y = max_z_point.y;
                slope_z = max_z_point.z;
            }
            else {
                slope_x = min_z_point.x;
                slope_y = min_z_point.y;
                slope_z = min_z_point.z;				
            }

            float slope_x_z = acos(std::sqrt(slope_x * slope_x / (slope_x * slope_x + slope_z * slope_z))) * 180 / M_PI;
            float slope_y_z = acos(std::sqrt(slope_x * slope_x / (slope_x * slope_x + slope_z * slope_z))) * 180 / M_PI;
            
            if (det_y == 0 || det_x == 0) {
                *output_obstacle += *cloud_cluster;
            }

            else if (max_z < -0.1 || min_z > 0.1) {
                *output_obstacle += *cloud_cluster;
            }
            else if (point_count != 0 && static_cast<float>(point_count_neg) / static_cast<float>(point_count) > 0.5) {
                *output_obstacle += *cloud_cluster;
            }
           
            else if (slope_x_z > 20 || slope_y_z > 40 || det_z / det_x > 0.7 || det_z / det_y > 0.7 || max_slope > 0.26) {
                if (det_x > 0.3 && det_z > 0.25) {
                    bool seg_flag = false;
                    std::sort(cloud_cluster->points.begin(), cloud_cluster->points.end(), std::bind(customCompare, std::placeholders::_1, std::placeholders::_2, R, T));
                    if (from_left_lidar_) {
                        std::reverse(cloud_cluster->points.begin(), cloud_cluster->points.end()); 
                    }

                    for (size_t i = 0; i < cloud_cluster->points.size(); i++) {
                        for (size_t j = i + 1; j < cloud_cluster->points.size(); j++) {
                            // 计算每个点和下一个点的距离
                            double distance = std::sqrt(
                                std::pow(cloud_cluster->points[i].x - cloud_cluster->points[j].x, 2) +
                                std::pow(cloud_cluster->points[i].y - cloud_cluster->points[j].y, 2) +
                                std::pow(cloud_cluster->points[i].z - cloud_cluster->points[j].z, 2)
                            );
                            if (distance > 0.15) {
                                double det_x_near = abs(cloud_cluster->points[j].x - cloud_cluster->points[i].x);
                                double det_y_near = abs(cloud_cluster->points[j].y - cloud_cluster->points[i].y);
                                double det_z_near = abs(cloud_cluster->points[j].z - cloud_cluster->points[i].z);

                                if (det_x_near == 0 || det_y_near == 0 || det_z_near / det_x_near > 0.7 || det_z_near / det_y_near > 0.7) {
                                    PointCloud::Ptr obstacle_cluster(new PointCloud);
                                    PointCloud::Ptr ground_cluster(new PointCloud);
                                    for (size_t k = 0; k < cloud_cluster->points.size(); k++) {
                                        if (k <= i) {
                                            ground_cluster->points.push_back(cloud_cluster->points[k]);
                                        }
                                        else {
                                            obstacle_cluster->points.push_back(cloud_cluster->points[k]);
                                        }
                                    }

                                    *output_obstacle += *obstacle_cluster;
                                    *output_ground += *ground_cluster;
                                    seg_flag = true;
            
                                }
                                break;
                            }
                        }
                        if (seg_flag) {
                            break;
                        }
                    }
                }

                else {
                    *output_obstacle += *cloud_cluster;
                }  
            }

            else {
                std::sort(cloud_cluster->points.begin(), cloud_cluster->points.end(), std::bind(customCompare, std::placeholders::_1, std::placeholders::_2, R, T));
                if (from_left_lidar_) {
                   std::reverse(cloud_cluster->points.begin(), cloud_cluster->points.end()); 
                }
                PointT start_point = cloud_cluster->points.front();
                // 初始化end_point （0，0，0）
                PointT end_point(0, 0, 0);
                for (size_t i = 1; i < cloud_cluster->points.size(); i++) {
                    // 计算每个点和start_point的距离
                    double distance = std::sqrt(
                        std::pow(cloud_cluster->points[i].x - start_point.x, 2) +
                        std::pow(cloud_cluster->points[i].y - start_point.y, 2) +
                        std::pow(cloud_cluster->points[i].z - start_point.z, 2)
                    );
                    if (distance > 0.1) {
                        end_point = cloud_cluster->points[i];
                        break; // 找到第一个距离大于0.15的点
                    }
                }

                if (end_point.x == 0) {
                    // 如果没有找到距离大于0.15的点，则使用最后一个点作为end_point
                    *output_ground += *cloud_cluster;
                }
                else {
                    // 计算end_point 和 start_point 的斜率
                    double det_x_near = abs(end_point.x - start_point.x);
                    double det_y_near = abs(end_point.y - start_point.y);
                    double det_z_near = abs(end_point.z - start_point.z);

                    bool is_all_neg = start_point.z < -0.15 && end_point.z < -0.15 && start_point.x < 1 && start_point.x > 0;
                    bool is_all_pos = start_point.z > -0.01 && end_point.z > 0.05 && start_point.x < 1 && start_point.x > 0;
                    if (det_x_near == 0 || det_y_near == 0) {
                        *output_obstacle += *cloud_cluster;
                    }
                    else if ((is_all_neg || is_all_pos) && (det_z_near / det_x_near > 0.6 || det_z_near / det_y_near > 0.7)) {
                        *output_obstacle += *cloud_cluster;
                    }
                    else {
                        if (is_ground_points) {
                            *output_ground += *cloud_cluster;
                        }
                        else {
                            // 如果不是地面点，计算地面距离0.15的地面点
                            // 初始化end_ground_point （0，0，0）
                            PointT end_ground_point(0, 0, 0);
                            std::vector<PointT> ground_points_vec;
                            for (size_t i = 1; i < ground_points->points.size(); i++) {
                                // 计算每个点和start_point的距离
                                double distance = std::sqrt(
                                    std::pow(ground_points->points[i].x - start_point.x, 2) +
                                    std::pow(ground_points->points[i].y - start_point.y, 2) +
                                    std::pow(ground_points->points[i].z - start_point.z, 2)
                                );
                                if (distance > 0.1 && distance < 0.15) {
                                    ground_points_vec.push_back(ground_points->points[i]);
                                }
                            }
                            std::sort(ground_points_vec.begin(), ground_points_vec.end(), 
                                [](const PointT& a, const PointT& b) {
                                    return a.x < b.x;
                                });
                            if (ground_points_vec.size() > 0) {
                                end_ground_point = ground_points_vec.front();
                            }
                            if (end_ground_point.x == 0 ) {
                                // 如果没有找到距离大于0.15的点，输出为地面点
                                if (is_all_neg) {
                                    *output_obstacle += *cloud_cluster;
                                    RCLCPP_INFO(logger_, "增加为障碍物");
                                }
                                else {
                                    *output_ground += *cloud_cluster;
                                }
                            }
                            else {
                                // end_ground_point 和 start_point 的斜率
                                det_x_near = abs(end_ground_point.x - start_point.x);
                                det_y_near = abs(end_ground_point.y - start_point.y);
                                det_z_near = abs(end_ground_point.z - start_point.z);
                                if (det_x_near == 0 || det_y_near == 0) {
                                    *output_obstacle += *cloud_cluster;
                                }
                                else if (det_z_near / det_x_near > 0.6 || det_z_near / det_y_near > 0.7) {
                                    *output_obstacle += *cloud_cluster;
                                }
                                else {
                                    *output_ground += *cloud_cluster;
                                }    
                            }
                        }
                    }
                }
            } 
        } 
    }
}

void Lidar_Obstacle_GroundModel::get_base_link_matrix(std::string flie_name,
    double& Tz, double& Ty, double& Tx, double& Rz, double& Ry, double& Rx, double& sort_angle)
{

    std::ifstream infile;
    infile.open(flie_name, std::ios::in);
    if (!infile.is_open()) {
        RCLCPP_ERROR(logger_, "get_base_link_matrix infile: %ld ms %s", errno, flie_name.c_str());
        return;
    }
    std::vector<double> v;
    std::string buf;
    while (getline(infile, buf)) {
        v.push_back(std::stod(buf));
    }
    infile.close();
    Tz = v[0];
    Ty = v[1];
    Tx = v[2];

    Rz = v[3];
    Ry = v[4];
    Rx = v[5];

    sort_angle = v[6];
}


void Lidar_Obstacle_GroundModel::read_from_yaml(std::string flie_name, bool is_left_lidar_, bool is_right_lidar_)
{
    YAML::Node config = YAML::LoadFile(flie_name);
    if (!config) {
        RCLCPP_ERROR(logger_, "read_from_yaml infile: %ld ms %s", errno, flie_name.c_str());
        return;
    }

   if (config["ground_model"] && config["ground_model"]["ranges"])
   { 
        const auto& ranges = config["ground_model"]["ranges"][0];
        double x = ranges["x"].as<double>();
        double y = ranges["y"].as<double>();
        double det_x = ranges["det_x"].as<double>();
        double det_y = ranges["det_y"].as<double>();
        double k;
        double b;
        if (det_x == 0) {
            k = 100000.0;
            b = x;
        }
        else {
            k = det_y / det_x;
            b = y - k * x;
        }

        if (is_left_lidar_) {
            left_k_ = k;
            left_b_ = b;
            RCLCPP_INFO(logger_, "left_k_: %f, left_b_: %f", left_k_, left_b_);
        } 
        if (is_right_lidar_) {
            right_k_ = k;
            right_b_ = b;
            RCLCPP_INFO(logger_, "right_k_: %f, right_b_: %f", right_k_, right_b_);
        }
   }

   if (config["ground_model"] && config["ground_model"]["start_point_side"])
   { 
        const auto& start_point_side = config["ground_model"]["start_point_side"][0];
        double start_point_side_x = start_point_side["x"].as<double>();
        double start_point_side_y = start_point_side["y"].as<double>();
        double start_point_side_z = start_point_side["z"].as<double>();
        PointT pcl_start_point_side;
        pcl_start_point_side.x = start_point_side_x;
        pcl_start_point_side.y = start_point_side_y;
        pcl_start_point_side.z = start_point_side_z;

        if (is_left_lidar_) {
            pcl_start_point_side_left_ = pcl_start_point_side;
            RCLCPP_INFO(logger_, "pcl_start_point_side_left_: %f, %f, %f", pcl_start_point_side_left_.x, pcl_start_point_side_left_.y, pcl_start_point_side_left_.z);
        }
        if (is_right_lidar_) {
            pcl_start_point_side_right_ = pcl_start_point_side;
            RCLCPP_INFO(logger_, "pcl_start_point_side_right_: %f, %f, %f", pcl_start_point_side_right_.x, pcl_start_point_side_right_.y, pcl_start_point_side_right_.z);
        }
    }

    if (config["ground_model"] && config["ground_model"]["end_point_side"]) 
    { 

        const auto& end_point_side = config["ground_model"]["end_point_side"][0];
        double end_point_side_x = end_point_side["x"].as<double>();
        double end_point_side_y = end_point_side["y"].as<double>();
        double end_point_side_z = end_point_side["z"].as<double>();
        PointT pcl_end_point_side;
        pcl_end_point_side.x = end_point_side_x;
        pcl_end_point_side.y = end_point_side_y;
        pcl_end_point_side.z = end_point_side_z;
        if (is_left_lidar_) {
            pcl_end_point_side_left_ = pcl_end_point_side;
            RCLCPP_INFO(logger_, "pcl_end_point_side_left_: %f, %f, %f", pcl_end_point_side_left_.x, pcl_end_point_side_left_.y, pcl_end_point_side_left_.z);
        }
        if (is_right_lidar_) {
            pcl_end_point_side_right_ = pcl_end_point_side;
            RCLCPP_INFO(logger_, "pcl_end_point_side_right_: %f, %f, %f", pcl_end_point_side_right_.x, pcl_end_point_side_right_.y, pcl_end_point_side_right_.z);
        }
    }


    if (config["ground_model"] && config["ground_model"]["segment_counts_side"]) 
    { 
        const auto& segment_counts_side = config["ground_model"]["segment_counts_side"];
        std::vector<int> segment_counts_side_ref;
        for (size_t i = 0; i < segment_counts_side.size(); ++i) {
            segment_counts_side_ref.push_back(segment_counts_side[i].as<int>());
            
        }
        if (is_left_lidar_) {
            segment_counts_side_left_ref_.clear();
            segment_counts_side_left_ref_ = segment_counts_side_ref;
            RCLCPP_INFO(logger_, "segment_counts_side_left_ref_: %d, %d, %d, %d, %d", segment_counts_side_left_ref_[0], segment_counts_side_left_ref_[1], segment_counts_side_left_ref_[2], segment_counts_side_left_ref_[3], segment_counts_side_left_ref_[4]);
        }
        if (is_right_lidar_) {
            segment_counts_side_right_ref_.clear();
            segment_counts_side_right_ref_ = segment_counts_side_ref;
            RCLCPP_INFO(logger_, "segment_counts_side_right_ref_: %d, %d, %d, %d, %d", segment_counts_side_right_ref_[0], segment_counts_side_right_ref_[1], segment_counts_side_right_ref_[2], segment_counts_side_right_ref_[3], segment_counts_side_right_ref_[4]);
        }
    }

    if (config["ground_model"] && config["ground_model"]["start_point_front"])
   { 
        const auto& start_point_front = config["ground_model"]["start_point_front"][0];
        double start_point_front_x = start_point_front["x"].as<double>();
        double start_point_front_y = start_point_front["y"].as<double>();
        double start_point_front_z = start_point_front["z"].as<double>();
        PointT pcl_start_point_front;
        pcl_start_point_front.x = start_point_front_x;
        pcl_start_point_front.y = start_point_front_y;
        pcl_start_point_front.z = start_point_front_z;
        if (is_left_lidar_) {
            pcl_start_point_front_left_ = pcl_start_point_front;
            RCLCPP_INFO(logger_, "pcl_start_point_front_left_: %f, %f, %f", pcl_start_point_front_left_.x, pcl_start_point_front_left_.y, pcl_start_point_front_left_.z);
        }
        if (is_right_lidar_) {
            pcl_start_point_front_right_ = pcl_start_point_front;
            RCLCPP_INFO(logger_, "pcl_start_point_front_right_: %f, %f, %f", pcl_start_point_front_right_.x, pcl_start_point_front_right_.y, pcl_start_point_front_right_.z);
        }

    }

    if (config["ground_model"] && config["ground_model"]["end_point_front"])
    { 
         const auto& end_point_front = config["ground_model"]["end_point_front"][0];
         double end_point_front_x = end_point_front["x"].as<double>();
         double end_point_front_y = end_point_front["y"].as<double>();
         double end_point_front_z = end_point_front["z"].as<double>();
         PointT pcl_end_point_front;
         pcl_end_point_front.x = end_point_front_x;
         pcl_end_point_front.y = end_point_front_y;
         pcl_end_point_front.z = end_point_front_z;
         if (is_left_lidar_) {
            pcl_end_point_front_left_ = pcl_end_point_front;
            RCLCPP_INFO(logger_, "pcl_end_point_front_left_: %f, %f, %f", pcl_end_point_front_left_.x, pcl_end_point_front_left_.y, pcl_end_point_front_left_.z);
         }
         if (is_right_lidar_) {
            pcl_end_point_front_right_ = pcl_end_point_front;
            RCLCPP_INFO(logger_, "pcl_end_point_front_right_: %f, %f, %f", pcl_end_point_front_right_.x, pcl_end_point_front_right_.y, pcl_end_point_front_right_.z);
         }

    }

    
    if (config["ground_model"] && config["ground_model"]["segment_counts_front"]) 
    { 
        const auto& segment_counts_front = config["ground_model"]["segment_counts_front"];
        std::vector<int> segment_counts_front_ref;
        for (size_t i = 0; i < segment_counts_front.size(); ++i) {
            segment_counts_front_ref.push_back(segment_counts_front[i].as<int>());
            
        }
        if (is_left_lidar_) {
            segment_counts_front_left_ref_.clear();
            segment_counts_front_left_ref_ = segment_counts_front_ref;
            RCLCPP_INFO(logger_, "segment_counts_front_left_ref_: %d, %d", segment_counts_front_left_ref_[0], segment_counts_front_left_ref_[1]);
        }
        if (is_right_lidar_) {
            segment_counts_front_right_ref_.clear();
            segment_counts_front_right_ref_ = segment_counts_front_ref;
            RCLCPP_INFO(logger_, "segment_counts_front_right_ref_: %d, %d", segment_counts_front_right_ref_[0], segment_counts_front_right_ref_[1]);
        }
    }

    
}

bool Lidar_Obstacle_GroundModel::check_data(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, float check_range, int min_num)
{
    int count = min_num;
    // 统计雷达 0.25 范围内点数
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
        float range = scan_msg->ranges[i]; // 获取当前点的距离
        // 如果距离有效（大于最小距离且小于最大距离）
        if (range >= scan_msg->range_min) {
            if (range < check_range) {
                count -= 1;
            }
        }
    }
    // count == min_num //   check_range 以内没有点，认为没有噪点
    // count < 0  // check_range 以内很多点，认为没有噪点
    return count == min_num || count < 0; // True 没有噪点
}


void Lidar_Obstacle_GroundModel::get_obstacle(PointCloud::Ptr obstacle_coarse, PointCloud::Ptr ground_coarse,
    PointCloud::Ptr obstacle_res, PointCloud::Ptr ground_res,
    const double sort_angle, const Eigen::Matrix4d TransMatrix, 
    const std::vector<int>& segment_counts_side_ref_, const std::vector<int>& segment_counts_front_ref_,
    const PointT& pcl_start_point_side, const PointT& pcl_end_point_side,
    const PointT& pcl_start_point_front, const PointT& pcl_end_point_front, bool from_left_lidar_) {

    PointCloud::Ptr obstacle_cluster(new PointCloud);
    PointCloud::Ptr output_ground(new PointCloud);

    // 转回laser_scan 坐标，进行按angle排序
    double R = static_cast<double>(sort_angle / 180.0 * M_PI);
    Eigen::Matrix3d RotateMatrix_sort;
    RotateMatrix_sort = Eigen::AngleAxisd(R, Eigen::Vector3d::UnitZ());
    RotateMatrix_sort = RotateMatrix_sort * TransMatrix.block<3, 3>(0, 0).inverse();
    Eigen::Vector3d TransVector_sort;
    TransVector_sort = TransMatrix.block<3, 1>(0, 3);

    // 对初始障碍物进行聚类识别，输出障碍物和地面
    get_min_cluster_filter(obstacle_coarse, obstacle_cluster,  output_ground, 0.1, 1, 2, RotateMatrix_sort, TransVector_sort, ground_coarse, false, from_left_lidar_);
    *ground_coarse += *output_ground;


    PointCloud::Ptr obstacle_ground(new PointCloud);
    PointCloud::Ptr output_ground_final(new PointCloud);
    // 对初始地面进行聚类识别，输出障碍物和地面
    get_min_cluster_filter(ground_coarse, obstacle_ground,  output_ground_final, 0.1, 1, 2, RotateMatrix_sort, TransVector_sort, ground_coarse, true, from_left_lidar_);
    *obstacle_cluster += *obstacle_ground;

    // 对障碍物和地面进行进一步处理,避免坡道影响
    PointCloud::Ptr obstacle_res_process(new PointCloud);
    PointCloud::Ptr ground_res_process(new PointCloud);
    judge_near_point(obstacle_cluster, output_ground_final,
    obstacle_res_process, ground_res_process, 0.2, 1, 2, 
    RotateMatrix_sort, TransVector_sort);

    std::vector<PointCloud::Ptr> obstacle_clusters_vector;
    obstacle_clusters_vector =  get_clusters(obstacle_res_process, 0.2, 3, 2);
    obstacle_res_process->clear();
    for (size_t i = 0; i < obstacle_clusters_vector.size(); i++) {
        for (size_t j = 0; j < obstacle_clusters_vector[i]->points.size(); j++) {
            PointT point_ob = obstacle_clusters_vector[i]->points[j];
            if (point_ob.z < -0.05 || point_ob.z > 0.05) {
                obstacle_res_process->points.push_back(point_ob);
            }
            else {
                ground_res_process->points.push_back(point_ob);
            }
        }
    }

    obstacle_cluster.swap(obstacle_res_process);
    output_ground_final.swap(ground_res_process);

    // 遍历障碍物点，统计z值小于 -0.1 的障碍物点的个数
    int count_z_neg = 0;
    for (const auto& point : obstacle_cluster->points) {
        if (point.z < -0.1) {
            count_z_neg++;
        }
    }
    if (count_z_neg > 10) {
        std::sort(output_ground_final->points.begin(), output_ground_final->points.end(), std::bind(customCompare, std::placeholders::_1, std::placeholders::_2, RotateMatrix_sort, TransVector_sort));
        // 将地面的端点标记为障碍物
        std::vector<std::vector<int>> point_index;
        for (size_t i = 1; i < output_ground_final->points.size(); i++) {
            std::vector<int> temp;
            float det_x_adj_point = output_ground_final->points[i].x - output_ground_final->points[i - 1].x;
            float det_y_adj_point = output_ground_final->points[i].y - output_ground_final->points[i - 1].y;
            float det_z_adj_point = output_ground_final->points[i].z - output_ground_final->points[i - 1].z;
            if (abs(det_x_adj_point) > 0.2 || 
            abs(output_ground_final->points[i].z - output_ground_final->points[i - 1].z) > 0.1 ) {
                temp.push_back(i - 1);
                temp.push_back(i);
                point_index.push_back(temp);
            }
        }

        if (point_index.size() == 0 && output_ground_final->points.size() != 0) {
            if (output_ground_final->points[0].z > -0.05 && output_ground_final->points[0].z < 0.05) {
                // 如果地面点的第一个点或最后一个点的z值小于-0.1，则将第一个点和最后一个点作为障碍物
                obstacle_cluster->points.push_back(output_ground_final->points[0]);
            }
            if (output_ground_final->points[output_ground_final->points.size() - 1].z > -0.05 && output_ground_final->points[output_ground_final->points.size() - 1].z < 0.05) {
                obstacle_cluster->points.push_back(output_ground_final->points[output_ground_final->points.size() - 1]);
            }
        }

        std::vector<std::vector<int>> point_index_four;

        for (size_t i = 0; i < point_index.size(); i++) {
            std::vector<int> temp;
            if (i == 0) {
                temp.push_back(0);
            } else {
                temp.push_back(point_index[i - 1][1]);
            }
            temp.push_back(point_index[i][0]);
            temp.push_back(point_index[i][1]);

            if (i == point_index.size() - 1) {
                temp.push_back(output_ground_final->points.size() - 1);
            } else {
                temp.push_back(point_index[i + 1][0]);
            }
            point_index_four.push_back(temp);
        }

        for (size_t i = 0; i < point_index_four.size(); i++) {
            if (output_ground_final->points[point_index_four[i][0]].z > -0.05 && output_ground_final->points[point_index_four[i][0]].z < 0.05) {
                obstacle_cluster->points.push_back(output_ground_final->points[point_index_four[i][0]]);
            }
            if (output_ground_final->points[point_index_four[i][1]].z > -0.05 && output_ground_final->points[point_index_four[i][1]].z < 0.05) {
                obstacle_cluster->points.push_back(output_ground_final->points[point_index_four[i][1]]);
            }
            
            if (i == point_index_four.size() - 1) {
                if (output_ground_final->points[point_index_four[i][2]].z > -0.05 && output_ground_final->points[point_index_four[i][2]].z < 0.05) {
                    obstacle_cluster->points.push_back(output_ground_final->points[point_index_four[i][2]]);
                }
                if (output_ground_final->points[point_index_four[i][3]].z > -0.05 && output_ground_final->points[point_index_four[i][3]].z < 0.05) {
                    obstacle_cluster->points.push_back(output_ground_final->points[point_index_four[i][3]]);
                }
            }
        } 
    }

    get_blind_spot_side(output_ground_final,
    pcl_start_point_side, pcl_end_point_side , obstacle_cluster);
    // get_blind_spot_front(output_ground_final,
    // pcl_start_point_side, pcl_end_point_side , obstacle_cluster);
    
    // // 盲区障碍物位置预估
    // std::vector<int> segment_counts(segment_counts_side_ref_.size(), 0);
    // countPointsInSegments(pcl_start_point_side, pcl_end_point_side, output_ground_final, segment_counts);
    // std::vector<float> segment_count_ratios;

    // for (size_t i = 0; i < segment_counts.size(); ++i) {
    //     float ratio = static_cast<float>(segment_counts[i]) / segment_counts_side_ref_[i];
    //     segment_count_ratios.push_back(ratio);
    // }

    // float min_ratio = segment_count_ratios[0];
    // int min_index = 0;
    // int max_index = segment_count_ratios.size() - 1;
    // for (size_t i = 0; i < segment_count_ratios.size(); ++i) {
    //     if (segment_count_ratios[i] < min_ratio) {
    //         min_ratio = segment_count_ratios[i];
    //         min_index = i;
    //     }
    //     else if (segment_count_ratios[i] == min_ratio) {
    //         max_index = i;
    //     }
    // }
    // // 侧面盲区
    // if (min_ratio < 0.2) {
    //     // 侧面没有地面点，侧面盲区
    //     if (min_index ==0) {
    //         double side_obstacle_y = pcl_start_point_side.y - ((pcl_start_point_side.y - pcl_end_point_side.y) / segment_count_ratios.size() * max_index * 1.2);
    //         if (pcl_start_point_side.y >0 && side_obstacle_y < 0.05) {
    //             side_obstacle_y = 0.05;
    //         }
    //         if (pcl_start_point_side.y <0 && side_obstacle_y > -0.05) {
    //             side_obstacle_y = -0.05;
    //         }
    //         for (double x = -0.1; x <= 0.1 ; x+= 0.05) {
    //             PointT temp;
    //             temp.x = x;
    //             temp.y = side_obstacle_y;
    //             temp.z = 0;
    //             obstacle_cluster->points.push_back(temp);
    //         }
    //     }  
    // }

    // // // 正面盲区
    // // if (min_ratio < 0.2) {
    // //     // 侧面没有地面点
    // //     if (min_index == 0 && max_index==4) {
    // //         std::vector<int> segment_counts_front(segment_counts_front_ref_.size(), 0);
    // //         countPointsInSegments(pcl_start_point_front, pcl_end_point_front, output_ground_final, segment_counts_front);
    // //         std::vector<float> segment_count_ratios_front;
    // //         for (size_t i = 0; i < segment_counts_front.size(); ++i) {
    // //             float ratio = static_cast<float>(segment_counts_front[i]) / segment_counts_front_ref_[i];
    // //             segment_count_ratios_front.push_back(ratio);
    // //         }
    // //         float min_ratio_front = segment_count_ratios_front[0];
    // //         int min_index_front = 0;
    // //         int max_index_front = segment_count_ratios_front.size() - 1;
    // //         for (size_t i = 0; i < segment_count_ratios_front.size(); ++i) {
    // //             if (segment_count_ratios_front[i] < min_ratio_front) {
    // //                 min_ratio_front = segment_count_ratios_front[i];
    // //                 min_index_front = i;
    // //             }
    // //             else if (segment_count_ratios_front[i] == min_ratio_front) {
    // //                 max_index_front = i;
    // //             }
    // //         }
    // //         if (min_ratio_front < 0.2) {
                
    // //             if (min_index_front == 0) {
    // //                 double front_obstacle_x = pcl_start_point_front.x -
    // //                 (sqrt((std::pow((pcl_end_point_front.x - pcl_start_point_front.x),2) + std::pow((pcl_end_point_front.y - pcl_start_point_front.y), 2)) 
    // //                 ) / segment_count_ratios_front.size() * (max_index_front+1) * 1.2);

    // //                 if (front_obstacle_x < 0.05) {
    // //                     front_obstacle_x = 0.05;
    // //                 }
    // //                 for (double y = -0.1; y <= 0.1 ; y+= 0.05) {
    // //                     PointT temp;
    // //                     temp.x = front_obstacle_x;
    // //                     temp.y = y;
    // //                     temp.z = 0;
    // //                     obstacle_cluster->points.push_back(temp);
    // //                 }
    // //             }

    // //         }   
    // //     }
    // //     // 侧面有地面点
    // //     else if(min_index >0 && max_index==4) {
    // //         double front_obstacle_x = pcl_start_point_side.x + ((pcl_end_point_side.x - pcl_start_point_side.x) / segment_count_ratios.size() * min_index);
    // //         if (front_obstacle_x < 0.05) {
    // //             front_obstacle_x = 0.05;
    // //         }
    // //         for (double y = -0.1; y <= 0.1 ; y+= 0.05) {
    // //             PointT temp;
    // //             temp.x = front_obstacle_x;
    // //             temp.y = y;
    // //             temp.z = 0;
    // //             obstacle_cluster->points.push_back(temp);
    // //         }
    // //     }
    // // }

    *obstacle_res += *obstacle_cluster;
    obstacle_res->height = 1;
    obstacle_res->width = obstacle_res->points.size();  
    obstacle_res->is_dense = false;

    *ground_res += *output_ground_final;
    ground_res->height = 1;
    ground_res->width = ground_res->points.size();
    ground_res->is_dense = false;
}

void Lidar_Obstacle_GroundModel::judge_near_point(PointCloud::Ptr obstacle_all, PointCloud::Ptr ground_all,
    PointCloud::Ptr obstacle_filter, PointCloud::Ptr ground_res, double r, int min_points, int mode, 
    const Eigen::Matrix3d R, const Eigen::Vector3d T) {

    std::vector<pcl::PointIndices> cluster_indices;
    int max_points = 1000; // 3

    *ground_res += *ground_all;
    if (obstacle_all->size() <= 3) {
        *obstacle_filter += *obstacle_all;
        RCLCPP_INFO(this->get_logger(), "obstacle_all size is 0");
        return;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(obstacle_all); // 创建点云索引向量，用于存储实际的点云信息
    if (mode == 1) {
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(r); // 设置近邻搜索的搜索半径为0.03m
        ec.setMinClusterSize(min_points); // 设置一个聚类需要的最少点数目为15
        ec.setMaxClusterSize(max_points); // 设置一个聚类需要的最大点数目为25000
        ec.setSearchMethod(tree); // 设置点云的搜索机制
        ec.setInputCloud(obstacle_all); //
        ec.extract(cluster_indices); // 从点云中提取聚类，并将点云索引保存在cluster_indices中/**/
    } else if (mode == 2) {
        DBSCANSimpleCluster<pcl::PointXYZ> ec;
        ec.setCorePointMinPts(1); // 3
        ec.setClusterTolerance(r);
        ec.setMinClusterSize(min_points);
        ec.setMaxClusterSize(max_points);
        ec.setSearchMethod(tree);
        ec.setInputCloud(obstacle_all);
        ec.extract(cluster_indices);
    }

    std::vector<PointCloud::Ptr> clusters_all;

    for (const auto& indices : cluster_indices) {
        PointCloud::Ptr cluster_xx(new PointCloud);
        for (int idx : indices.indices) {
            cluster_xx->points.push_back(obstacle_all->points[idx]);
        }
        std::sort(cluster_xx->points.begin(), cluster_xx->points.end(),
                [](const PointT& a, const PointT& b) {
                    return a.x < b.x;
                });

        clusters_all.push_back(cluster_xx);
    }

    std::sort(clusters_all.begin(), clusters_all.end(),
          [](const PointCloud::Ptr& a, const PointCloud::Ptr& b) {
              if (a->empty()) return false;
              if (b->empty()) return true;
              return a->points.front().x < b->points.front().x;
          });

    for (auto& cloud_cluster: clusters_all) {
        // 只有一个点的情况
        if (cloud_cluster->size() ==1) {
            if (cloud_cluster->points[0].z < -0.15) {
                PointT start_point =cloud_cluster->points[0];
                if (start_point.x < 2 && start_point.x > 0.5) {
                    PointT near_ground_point(0, 0, 0);
                    double min_distance = std::numeric_limits<double>::max();
                    std::vector<PointT> ground_points_vec;
                    for (size_t i = 0; i < ground_res->points.size(); i++) {
                        if (ground_res->points[i].x > start_point.x) {
                            continue;
                        }
                        // 计算每个点和start_point的距离
                        double distance = abs(ground_res->points[i].x - start_point.x);
                        if (distance < min_distance) {
                            min_distance = distance;
                            near_ground_point = ground_res->points[i];
                        }
                        if (distance > 0.15 && distance < 0.3) {
                            ground_points_vec.push_back(ground_res->points[i]);
                        }
                    }

                    std::sort(ground_points_vec.begin(), ground_points_vec.end(), [start_point](const PointT& a, const PointT& b) {
                        return std::sqrt(
                            std::pow(a.x - start_point.x, 2) +
                            std::pow(a.y - start_point.y, 2) +
                            std::pow(a.z - start_point.z, 2)) < std::sqrt(
                            std::pow(b.x - start_point.x, 2) +
                            std::pow(b.y - start_point.y, 2) +
                            std::pow(b.z - start_point.z, 2)); // 按照x坐标排序
                    });

                    PointT end_point(0, 0, 0);
                    if (ground_points_vec.size() > 0) {
                        // 取距离起始点最近的点作为end_point
                        end_point = ground_points_vec.front();
                    } 

                    // 如果都不存在 
                    if (end_point.x == 0 && near_ground_point.x ==0) {
                        *obstacle_filter += *cloud_cluster;
                    }
                    // 存在最近点
                    else if (end_point.x == 0 && near_ground_point.x != 0) {
                        // 如果没有找到距离大于0.15的点，则使用near_ground_point作为end_point
                        *obstacle_filter += *cloud_cluster;
                    }
                    // 都存在
                    else {
                        if (abs(near_ground_point.z - start_point.z) > 0.1) {
                            // 如果没有找到距离大于0.15的点，则使用end_point作为near_ground_point
                            *obstacle_filter += *cloud_cluster;
                        }
                        else {
                            // 计算end_point 和 start_point 的斜率
                            double det_x_near = abs(end_point.x - start_point.x);
                            double det_y_near = abs(end_point.y - start_point.y);
                            double det_z_near = abs(end_point.z - start_point.z);

                            if (det_x_near == 0 || det_y_near == 0) {
                                *obstacle_filter += *cloud_cluster;
                            }
                            else if (det_z_near / det_x_near > 0.6 || det_z_near / det_y_near > 0.7) {
                                *obstacle_filter += *cloud_cluster;
                            }
                            else {
                                *ground_res += *cloud_cluster;
                            }
                        }
                        
                    }
                }
                else {
                    *obstacle_filter += *cloud_cluster;
                }
            }
            else {
                *ground_res += *cloud_cluster;
            }
        }
        else {
            float min_z, max_z, min_x, max_x, min_y, max_y;
            PointT min_z_point, max_z_point;
            min_z = cloud_cluster->points[0].z;
            max_z = cloud_cluster->points[0].z;
            min_x = cloud_cluster->points[0].x;
            max_x = cloud_cluster->points[0].x;
            min_y = cloud_cluster->points[0].y;
            max_y = cloud_cluster->points[0].y;

            min_z_point = cloud_cluster->points[0];
            max_z_point = cloud_cluster->points[0];

            for (auto& single_point: cloud_cluster->points) {
                min_z = std::min(min_z, single_point.z);
                max_z = std::max(max_z, single_point.z);
                min_x = std::min(min_x, single_point.x);
                max_x = std::max(max_x, single_point.x);
                min_y = std::min(min_y, single_point.y);
                max_y = std::max(max_y, single_point.y);
                if (min_z == single_point.z) {
                    min_z_point = single_point;
                }

                if (max_z == single_point.z) {
                    max_z_point = single_point;
                }
            }
   
            if (min_x < 2 && min_x > 0.5 && (max_z < -0.1 || min_z > 0.1) && abs(max_x - min_x) > 0.2) {
                std::sort(cloud_cluster->points.begin(), cloud_cluster->points.end(), std::bind(customCompare, std::placeholders::_1, std::placeholders::_2, R, T));
                PointT point_1 = cloud_cluster->points.front();
                PointT point_2 = cloud_cluster->points.back();
                // 选取x绝对值小的点作为起始点
                PointT start_point = (std::abs(point_1.x) < std::abs(point_2.x)) ? point_1 : point_2;
                PointT near_ground_point(0, 0, 0);
                double min_distance = std::numeric_limits<double>::max();
                std::vector<PointT> ground_points_vec;
                for (size_t i = 0; i < ground_res->points.size(); i++) {
                    if (ground_res->points[i].x > start_point.x) {
                       continue;
                    }
                    // 计算每个点和start_point的距离
                    double distance = abs(ground_res->points[i].x - start_point.x);
                    if (distance < min_distance) {
                        min_distance = distance;
                        near_ground_point = ground_res->points[i];
                    }
                    if (distance > 0.15 && distance < 0.3) {
                        ground_points_vec.push_back(ground_res->points[i]);
                    }
                }

                std::sort(ground_points_vec.begin(), ground_points_vec.end(), [start_point](const PointT& a, const PointT& b) {
                    return std::sqrt(
                        std::pow(a.x - start_point.x, 2) +
                        std::pow(a.y - start_point.y, 2) +
                        std::pow(a.z - start_point.z, 2)) < std::sqrt(
                        std::pow(b.x - start_point.x, 2) +
                        std::pow(b.y - start_point.y, 2) +
                        std::pow(b.z - start_point.z, 2)); // 按照x坐标排序
                });

                PointT end_point(0, 0, 0);
                if (ground_points_vec.size() > 0) {
                    // 取距离起始点最近的点作为end_point
                    end_point = ground_points_vec.front();
                } 

                // 如果都不存在 
                if (end_point.x == 0 && near_ground_point.x ==0) {
                    *obstacle_filter += *cloud_cluster;
                }
                // 存在最近点
                else if (end_point.x == 0 && near_ground_point.x != 0) {
                    // 如果没有找到距离大于0.15的点，则使用near_ground_point作为end_point
                    *obstacle_filter += *cloud_cluster;
                }
                // 都存在
                else {
                    if (abs(near_ground_point.z - start_point.z) > 0.1) {
                        // 如果没有找到距离大于0.15的点，则使用end_point作为near_ground_point
                        *obstacle_filter += *cloud_cluster;
                    }
                    else {
                        // 计算end_point 和 start_point 的斜率
                        double det_x_near = abs(end_point.x - start_point.x);
                        double det_y_near = abs(end_point.y - start_point.y);
                        double det_z_near = abs(end_point.z - start_point.z);

                        if (det_x_near == 0 || det_y_near == 0) {
                            *obstacle_filter += *cloud_cluster;
                        }
                        else if (det_z_near / det_x_near > 0.6 || det_z_near / det_y_near > 0.7) {
                            *obstacle_filter += *cloud_cluster;
                        }
                        else {
                            *ground_res += *cloud_cluster;
                        }
                    }
                    
                }
            }
            else {
                *obstacle_filter += *cloud_cluster;
            }
        } 
    }
    
    obstacle_filter->height = 1;
    obstacle_filter->width = obstacle_filter->points.size();
    obstacle_filter->is_dense = false;
    ground_res->height = 1;
    ground_res->width = ground_res->points.size();
    ground_res->is_dense = false;
   
}


void Lidar_Obstacle_GroundModel::get_blind_spot_side(PointCloud::Ptr ground_pointcloud,
    const PointT& pcl_start_point, const PointT& pcl_end_point, PointCloud::Ptr obstacle_cluster) {

    PointCloud::Ptr output_all(new PointCloud);
    
    countPointsInSegments_All(pcl_start_point,
                        pcl_end_point,
                        ground_pointcloud,
                        output_all);

    
    double side_obstacle_y = 0;
    if (output_all->points.size() > 5) {
        PointCloud::Ptr output_(new PointCloud);
        get_max_cluster(output_all, output_, 0.05, 5, 2);
        
        std::sort(output_->points.begin(), output_->points.end(),
        [&](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
            return a.x < b.x;  // 按照 x 坐标排序
        });
    
    
        side_obstacle_y = output_->points.begin()->y * 1.3; // 取第一个点的y值作为侧面盲区障碍物的y值
        if (pcl_start_point.y >0 && side_obstacle_y < 0.05) {
            side_obstacle_y = 0.05;
        }
        if (pcl_start_point.y <0 && side_obstacle_y > -0.05) {
            side_obstacle_y = -0.05;
        }
        if (abs(side_obstacle_y) < abs(pcl_start_point.y)) {
            for (double x = -0.1; x <= 0.1 ; x+= 0.05) {
                PointT temp;
                temp.x = x;
                temp.y = side_obstacle_y;
                temp.z = 0;
                obstacle_cluster->points.push_back(temp);
            }
        }

    }
    else {
        if (pcl_start_point.y > 0 ) {
            side_obstacle_y = 0.05;
        }
        if (pcl_start_point.y < 0 ) {
            side_obstacle_y = -0.05;
        }
        for (double x = -0.1; x <= 0.1 ; x+= 0.05) {
            PointT temp;
            temp.x = x;
            temp.y = side_obstacle_y;
            temp.z = 0;
            obstacle_cluster->points.push_back(temp);
        }
    }
        
}


void Lidar_Obstacle_GroundModel::get_blind_spot_front(PointCloud::Ptr ground_pointcloud,
    const PointT& pcl_start_point, const PointT& pcl_end_point, PointCloud::Ptr obstacle_cluster) {

    PointCloud::Ptr output_all(new PointCloud);
    
    countPointsInSegments_All(pcl_start_point,
                        pcl_end_point,
                        ground_pointcloud,
                        output_all);

    
    double front_obstacle_x = 0;
    if (output_all->points.size() > 5) {
        PointCloud::Ptr output_(new PointCloud);
        get_max_cluster(output_all, output_, 0.05, 5, 2);
        
        std::sort(output_->points.begin(), output_->points.end(),
        [&](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
            return a.x > b.x;  // 按照 x 坐标排序
        });
    
    
        front_obstacle_x = output_->points.begin()->x * 1.3; // 取第一个点的x值作为侧面盲区障碍物的x值
        if (front_obstacle_x < 0.05) {
            front_obstacle_x = 0.05;
        }
        if (abs(front_obstacle_x) < abs(pcl_end_point.x)) {
            for (double y = -0.1; y <= 0.1 ; y+= 0.05) {
                PointT temp;
                temp.x = front_obstacle_x;
                temp.y = y;
                temp.z = 0;
                obstacle_cluster->points.push_back(temp);
            }
        }

    }
    else {
        front_obstacle_x = 0.05;
        for (double y = -0.1; y <= 0.1 ; y+= 0.05) {
            PointT temp;
            temp.x = front_obstacle_x;
            temp.y = y;
            temp.z = 0;
            obstacle_cluster->points.push_back(temp);
        }
    }
        
}

void Lidar_Obstacle_GroundModel::pedal_detect() {
    int pedal_state = last_alert_value_;
    // left pedal
    bool pedal_left_flag_ = false;
    std::vector<PointCloud::Ptr> left_pedal_clusters;
    left_pedal_clusters = get_clusters(cloud_for_pedal_left_, 0.05, 6, 2);
    for (auto& cloud_cluster: left_pedal_clusters) {
        float min_z, max_z, min_x, max_x, min_y, max_y;
        min_z = cloud_cluster->points[0].z;
        max_z = cloud_cluster->points[0].z;
        min_x = cloud_cluster->points[0].x;
        max_x = cloud_cluster->points[0].x;
        min_y = cloud_cluster->points[0].y;
        max_y = cloud_cluster->points[0].y;

        for (auto& single_point: cloud_cluster->points) {
            min_z = std::min(min_z, single_point.z);
            max_z = std::max(max_z, single_point.z);
            min_x = std::min(min_x, single_point.x);
            max_x = std::max(max_x, single_point.x);
            min_y = std::min(min_y, single_point.y);
            max_y = std::max(max_y, single_point.y);
        }
        if (min_z > 0.13 && min_y > -0.15 && max_y < 0.2 && max_x < 0.4 && min_x > 0) {
            pedal_left_flag_ = true;
            break;
        }
    }


    // right pedal
    bool pedal_right_flag_ = false;  
    std::vector<PointCloud::Ptr> right_pedal_clusters;
    right_pedal_clusters = get_clusters(cloud_for_pedal_right_, 0.05, 6, 2);
    for (auto& cloud_cluster: right_pedal_clusters) {
        float min_z, max_z, min_x, max_x, min_y, max_y;
        min_z = cloud_cluster->points[0].z;    
        max_z = cloud_cluster->points[0].z;
        min_x = cloud_cluster->points[0].x;
        max_x = cloud_cluster->points[0].x;
        min_y = cloud_cluster->points[0].y;
        max_y = cloud_cluster->points[0].y;   
        
        for (auto& single_point: cloud_cluster->points) {
            min_z = std::min(min_z, single_point.z);
            max_z = std::max(max_z, single_point.z);
            min_x = std::min(min_x, single_point.x);
            max_x = std::max(max_x, single_point.x);
            min_y = std::min(min_y, single_point.y);
            max_y = std::max(max_y, single_point.y);
        }
        
        if (min_z > 0.13 && min_y > -0.2 && max_y < 0.15 && max_x < 0.4 && min_x > 0) {
            pedal_right_flag_ = true;
            break; 
        }  
    }

    // 计算当前预警值
    int current_alert_value = 0;
    if (pedal_left_flag_ && pedal_right_flag_) {
        current_alert_value = 3; // 左右都有障碍物
    } else if (pedal_left_flag_) {
        current_alert_value = 1; // 只有左侧有障碍物
    } else if (pedal_right_flag_) {
        current_alert_value = 2; // 只有右侧有障碍物
    }


    if (pedal_state ==0) {
        if (current_alert_value != 0) {
            pedal_limit_count_++;

            if (pedal_limit_count_ > 5) {
                pedal_state = current_alert_value; 
                pedal_limit_count_ = 0; // 重置计数器
            }
        }
        else {
            pedal_limit_count_ = 0; // 重置计数器
        }
    }

    else {
        if (current_alert_value == 0) {
            pedal_normal_count_++;
            if (pedal_normal_count_ > 5) {
                pedal_state = 0; 
                pedal_normal_count_ = 0; // 重置计数器
            }
        }
        else {
            pedal_normal_count_ = 0; // 重置计数器
            if (current_alert_value != pedal_state) {
                pedal_limit_count2_ ++; 
                if (pedal_limit_count2_ > 5) {
                    pedal_state = current_alert_value; 
                    pedal_limit_count2_ = 0; // 重置计数器
                }
            }
            else {
                pedal_limit_count2_ = 0; // 重置计数器
            }
        } 
    }

    if (pedal_state != last_alert_value_) {
        std_msgs::msg::Int32 alert_msg;
        alert_msg.data = pedal_state;
        pedal_limiter_pub_->publish(alert_msg);
        // 打印预警值变化信息
        RCLCPP_INFO(this->get_logger(), "预警值变化: %d -> %d", last_alert_value_, pedal_state);
        last_alert_value_ = pedal_state;
    }
}