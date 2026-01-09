#include "podao_detect.h"
#include <iostream>
#include <queue>
#include <vector>

// // 计算两个向量的叉积
// float crossProduct(const Eigen::Vector2f& a, const Eigen::Vector2f& b) {
//     return a.x() * b.y() - a.y() * b.x();
// }

// // 计算点与原点的极角（相对于x轴的角度）
// float angleFromCenter(const Point& center, const Point& p) {
//     return std::atan2(p.y - center.y, p.x - center.x); // atan2会返回-π到π之间的角度
// }

// // 按照角度对点排序（逆时针）
// void sortPointsCounterClockwise(std::vector<Point>& points, Point center) {

//     // 按照角度对点进行排序
//     std::sort(points.begin(), points.end(), [&center](const Point& a, const Point& b) {
//         return angleFromCenter(center, a) < angleFromCenter(center, b);
//     });
// }

// // 判断点是否在旋转矩形内
// void isPointInRotatedRectangle(PointCloud::Ptr cloud, std::vector<Point>& rectangle_points, PointCloud::Ptr cloud_in, bool& is_in_rect) {

//     // 计算矩形的四个顶点
//     Eigen::Vector2f p1(rectangle_points[0].x, rectangle_points[0].y);  // 左下角
//     Eigen::Vector2f p2(rectangle_points[1].x, rectangle_points[1].y);   // 右下角
//     Eigen::Vector2f p3(rectangle_points[2].x, rectangle_points[2].y);    // 右上角
//     Eigen::Vector2f p4(rectangle_points[3].x, rectangle_points[3].y);   // 左上角
//     // 使用向量叉积判断点是否在矩形内
//     Eigen::Vector2f edge1 = p2 - p1;  // p1 -> p2 边
//     Eigen::Vector2f edge2 = p3 - p2;  // p2 -> p3 边
//     Eigen::Vector2f edge3 = p4 - p3;  // p3 -> p4 边
//     Eigen::Vector2f edge4 = p1 - p4;  // p4 -> p1 边

//     is_in_rect = false;
    
//     for (const auto& point : cloud->points) {
//         Eigen::Vector2f point_vector(point.x, point.y);
//         Eigen::Vector2f v1 = point_vector - p1; // 点到p1的向量
//         Eigen::Vector2f v2 = point_vector - p2; // 点到p2的向量
//         Eigen::Vector2f v3 = point_vector - p3; // 点到p3的向量
//         Eigen::Vector2f v4 = point_vector - p4; // 点到p4的向量

//             // 计算叉积，确保点在四个边的同一侧
//         if (crossProduct(edge1, v1) < 0 || crossProduct(edge2, v2) < 0 || 
//             crossProduct(edge3, v3) < 0 || crossProduct(edge4, v4) < 0) {

//         }
//         else {
//             is_in_rect = true;
//             cloud_in->points.push_back(point);
//         }
//     }
// }

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

static double pointToLineDistance(const pcl::PointXYZ& p, const pcl::PointXYZ& p0, const pcl::PointXYZ& d)
{
    double px = p.x - p0.x;
    double py = p.y - p0.y;
    double pz = p.z - p0.z;

    double cross_x = py * d.z - pz * d.y;
    double cross_y = pz * d.x - px * d.z;
    double cross_z = px * d.y - py * d.x;
    double cross_norm = std::sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
    double d_norm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z) + 1e-6; // 避免除零

    return cross_norm / d_norm;
}

// **RANSAC 直线拟合**
static void fitLineRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, double threshold, std::vector<double>& coefficients,
    pcl::PointIndices::Ptr& inlierIndices)
{

    pcl::PointXYZ p0;
    p0.x = coefficients[0];
    p0.y = coefficients[1];
    p0.z = coefficients[2];

    pcl::PointXYZ d;
    d.x = coefficients[3];
    d.y = coefficients[4];
    d.z = coefficients[5];
    // 归一化方向向量
    double norm = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    d.x /= norm;
    d.y /= norm;
    d.z /= norm;

    // **3. 计算内点**
    std::vector<int> inliers;
    for (int i = 0; i < cloud->size(); i++) {
        double dist = pointToLineDistance((*cloud)[i], p0, d);
        if (dist < threshold) {
            inliers.push_back(i);
        }
    }

    inlierIndices->indices = inliers;
}

bool isValidTransition(Status from, Status to) {
    switch (from) {
        case Flat:
            return to == Flat || to == Up || to == Down || to == FlatToUp || to == FlatToDown;
        case Up:
            return to == Up || to == Flat;
        case Down:
            return  to == Down || to == Flat;
        case FlatToUp:
            return to == FlatToUp ||to == Up || to == Flat;
        case FlatToDown:
            return to == FlatToDown || to == Down || to == Flat;
        default:
            return false;
    }
}


Podao_Detect::Podao_Detect(std::string name)
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

        get_base_link_matrix(path + "../../../../src/calibration_files/01/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);
    
    } 
    else if (mac == "66:FA:C4:30:2A:24") {
        get_base_link_matrix(path + "../../../../src/calibration_files/02/left_tilt_lidar.txt",
            left_z_, left_y_, left_x_, left_Rz1_, left_Ry1_, left_Rx1_, left_sort_angle_);

        get_base_link_matrix(path + "../../../../src/calibration_files/02/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);
    }
    else if (mac == "46:F9:C0:AD:42:4C") {
        get_base_link_matrix(path + "../../../../src/calibration_files/03/left_tilt_lidar.txt",
            left_z_, left_y_, left_x_, left_Rz1_, left_Ry1_, left_Rx1_, left_sort_angle_);
            
        get_base_link_matrix(path + "../../../../src/calibration_files/03/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);

        read_from_yaml(path + "../../../../src/calibration_files/03/imu.yaml");
    }
    else if (mac == "96:AF:3E:5F:A9:B8") {
        get_base_link_matrix(path + "../../../../src/calibration_files/04/left_tilt_lidar.txt",
            left_z_, left_y_, left_x_, left_Rz1_, left_Ry1_, left_Rx1_, left_sort_angle_);
            
        get_base_link_matrix(path + "../../../../src/calibration_files/04/right_tilt_lidar.txt",
            right_z_, right_y_, right_x_, right_Rz1_, right_Ry1_, right_Rx1_, right_sort_angle_);

        read_from_yaml(path + "../../../../src/calibration_files/04/imu.yaml");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "the ip is not correct");
        return;
    }

    // // publisher_
    // transform_pointcloud2_right_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/podao/transform_pointcloud2_right", 1);
    // transform_pointcloud2_left_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/podao/transform_pointcloud2_left", 1);
    // // 坡道位置
    // podao_pointcloud2_left_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/podao_pointcloud2_left", 1);
    // podao_pointcloud2_right_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/podao_pointcloud2_right", 1);
    
    // 创建 LaserScan 消息订阅者
    scan_subscriber_left_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftTiltLaserTopic, 10,
        std::bind(&Podao_Detect::scan_callback_left, this, std::placeholders::_1));
    scan_subscriber_right_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightTiltLaserTopic, 10,
        std::bind(&Podao_Detect::scan_callback_right, this, std::placeholders::_1));
    
    // 创建 水平LaserScan 消息订阅者
    // scan_subscriber_right_Hori_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     rightLaserTopic, 10,
    //     std::bind(&Podao_Detect::scan_callback_right_Hori, this, std::placeholders::_1));

    // scan_subscriber_left_Hori_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    //     leftLaserTopic, 10,
    //     std::bind(&Podao_Detect::scan_callback_left_Hori, this, std::placeholders::_1));

    scene_mode_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/scene_services/mode", 10, 
        std::bind(&Podao_Detect::listener_scene_mode_callback, this, std::placeholders::_1));

    imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/sensor/imu/data", 10,
        std::bind(&Podao_Detect::imu_callback, this, std::placeholders::_1));

    // timer_front_obstacle_ = this->create_wall_timer(
    //     std::chrono::milliseconds(100), std::bind(&Podao_Detect::front_obstacle_Callback, this));

    //publisher_
    status_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/perception/podao_detect_status", 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/perception/podao_detect_marker", 10);

    podao_line_vector_left_.clear();
    podao_line_vector_right_.clear();

}


void Podao_Detect::listener_scene_mode_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::string scene_mode = msg->data;
    if (scene_mode.find("narrow") == 0) {
        RCLCPP_INFO(this->get_logger(), "The current mode is: %s", scene_mode.c_str());
        is_in_narrow_mode_ = true;
    }

    else {
        is_in_narrow_mode_ = false;
    }

    RCLCPP_INFO(this->get_logger(), "is_in_narrow_mode_   %d", is_in_narrow_mode_);
}


void Podao_Detect::imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    // 处理 IMU 数据
    historyStatus_ = currentStatus_; // 保存历史状态
   
    tf2::Quaternion q(
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z,
        imu_msg->orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 转为角度
    double pitch_deg = roll * 180.0 / M_PI;

    pitch_deg -= pitch_base_;

    // 判定状态
    if (pitch_deg < -3.0 ) {
        down_count_ = 0; // 重置计数器
        flat_count_ = 0; // 重置计数器
        up_count_++;
        if (up_count_ > 15) {
            currentStatus_ = Up; // 更新状态为上坡
            //std::cout << "当前为上坡，Pitch = " << pitch_deg << "°" << std::endl;
        }
    } 
    
    else if (pitch_deg > 3.0) {
        up_count_ = 0; // 重置计数器
        flat_count_ = 0; // 重置计数器
        down_count_ ++;
        if (down_count_ > 15) {
            currentStatus_ = Down; // 更新状态为上坡
            //std::cout << "当前为下坡，Pitch = " << pitch_deg << "°" << std::endl;
        }
    }
    else {
        down_count_ = 0; // 重置计数器
        up_count_ = 0; // 重置计数器
        flat_count_++;
        if (flat_count_ > 15) {
            currentStatus_ = Flat; // 更新状态为平路
            //std::cout << "当前为平路，Pitch = " << pitch_deg << "°" << std::endl;
        }
    }

    if (currentStatus_ == Flat) {
        Status futureStatus_left = historyStatus_lidar_left_;
        if (podao_line_vector_left_.size() > 0) {
            PointCloud::Ptr podao_line_left = podao_line_vector_left_[0];
            float start_z = podao_line_left->points[0].z;
            float end_z = podao_line_left->points[podao_line_left->points.size() - 1].z;
            if (end_z - start_z > 0.07) {
                futureStatus_left_flat_count_ = 0;
                futureStatus_left_down_count_ = 0;
                futureStatus_left_up_count_++;
                if (futureStatus_left_up_count_ > 10) {
                    futureStatus_left = Up;
                }
            } 
            else if (end_z - start_z < -0.07) {
                futureStatus_left_up_count_ = 0;
                futureStatus_left_flat_count_ = 0;
                futureStatus_left_down_count_++;
                if (futureStatus_left_down_count_ > 10) {
                    futureStatus_left = Down;
                }
            } 
            else {
                futureStatus_left_up_count_ = 0;
                futureStatus_left_down_count_ = 0;
                futureStatus_left_flat_count_++;
                if (futureStatus_left_flat_count_ > 10) {
                    futureStatus_left = Flat;
                }
            } 
        }
        else {
            futureStatus_left_up_count_ = 0;
            futureStatus_left_down_count_ = 0;
            futureStatus_left_flat_count_++;
            if (futureStatus_left_flat_count_ > 10) {
                futureStatus_left = Flat;
            }
        }

        historyStatus_lidar_left_ = futureStatus_left; // 更新历史状态
        Status futureStatus_right = historyStatus_lidar_right_; // 初始化为历史状态
        if (podao_line_vector_right_.size() > 0) {
            PointCloud::Ptr podao_line_right = podao_line_vector_right_[0];
            float start_z = podao_line_right->points[0].z;
            float end_z = podao_line_right->points[podao_line_right->points.size() - 1].z;
            if (end_z - start_z > 0.07) {
                futureStatus_right_flat_count_ = 0;
                futureStatus_right_down_count_ = 0;
                futureStatus_right_up_count_++;
                if (futureStatus_right_up_count_ > 10) {
                    futureStatus_right = Up;
                }
            } 
            else if (end_z - start_z < -0.07) {
                futureStatus_right_up_count_ = 0;
                futureStatus_right_flat_count_ = 0;
                futureStatus_right_down_count_++;
                if (futureStatus_right_down_count_ > 10) {
                    futureStatus_right = Down;
                }
            } 
            else {
                futureStatus_right_up_count_ = 0;
                futureStatus_right_down_count_ = 0;
                futureStatus_right_flat_count_++;
                if (futureStatus_right_flat_count_ > 10) {
                    futureStatus_right = Flat;
                }
            }   
        }
        else {
            futureStatus_right_up_count_ = 0;
            futureStatus_right_down_count_ = 0;
            futureStatus_right_flat_count_++;
            if (futureStatus_right_flat_count_ > 10) {
                futureStatus_right = Flat;
            }
        }
        historyStatus_lidar_right_ = futureStatus_right; // 更新历史状态

        if (futureStatus_left == Up && futureStatus_right == Up) {
            currentStatus_ = FlatToUp;
        } 
        else if (futureStatus_left == Up && futureStatus_right == Flat) {
            currentStatus_ = FlatToUp;
        } 
        else if (futureStatus_left == Up && futureStatus_right == Down){
            currentStatus_ = Flat;
        }

        else if (futureStatus_left == Down && futureStatus_right == Up) {
            currentStatus_ = Flat;
        } 
        else if (futureStatus_left == Down && futureStatus_right == Flat) {
            currentStatus_ = FlatToDown;
        }
        else if (futureStatus_left == Down && futureStatus_right == Down) {
            currentStatus_ = FlatToDown;
        } 

        else if (futureStatus_left == Flat && futureStatus_right == Up) {
            currentStatus_ = FlatToUp;
        } 
        else if (futureStatus_left == Flat && futureStatus_right == Down) {
            currentStatus_ = FlatToDown;
        } 
        else if (futureStatus_left == Flat && futureStatus_right == Flat) {
            currentStatus_ = Flat;
        }
    }

    // 状态机
    if (currentStatus_ != Up && currentStatus_ != Down) {
        if (!isValidTransition(historyStatus_, currentStatus_)) {
            currentStatus_ = historyStatus_; // 如果状态转换不合法，保持当前状态
        }
    } 

    // 叠加多帧
    // 解决重复播报问题
    if (historyStatus_ == FlatToDown && currentStatus_ == Flat) {
        flattodown2flat_count_++;
        if (flattodown2flat_count_ > 20) {
            currentStatus_ = Flat; // 更新状态为平路
            flattodown2flat_count_ = 0; // 重置计数器
        }
        else {
            currentStatus_ = FlatToDown; // 保持当前状态
        }
    }
    else {
        flattodown2flat_count_ = 0; // 重置计数器
    }

    if (historyStatus_ == FlatToUp && currentStatus_ == Flat) {
        flattoup2flat_count_++;
        if (flattoup2flat_count_ > 20) {
            currentStatus_ = Flat; // 更新状态为平路
            flattoup2flat_count_ = 0; // 重置计数器
        }
        else {
            currentStatus_ = FlatToUp; // 保持当前状态
        }
    }
    else {
        flattoup2flat_count_ = 0; // 重置计数器
    }

    // 解决下坡时误报上坡问题
    if (historyStatus_ == Down && currentStatus_ == Flat) {
        down2flat_count_++;
        if (down2flat_count_ > 15) {
            currentStatus_ = Flat; // 更新状态为平路
            down2flat_count_ = 0; // 重置计数器
        }
        else {
            currentStatus_ = Down; // 保持当前状态
        }
    }
    else {
        down2flat_count_ = 0; // 重置计数器
    }

    if (is_in_narrow_mode_) {
        currentStatus_ = Flat;
    }


    // 发布状态信息
    std_msgs::msg::Int32 msg;
    msg.data = static_cast<int>(currentStatus_);
    status_publisher_->publish(msg);

    // 利用rivz 可视化，不同状态用不同颜色表示
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link_map"; // 你可以修改为 "base_link" 或其他坐标系
    marker.header.stamp = imu_msg->header.stamp; // 使用 IMU 消息的时间戳
    marker.ns = "podao_detect";
    marker.id = 0; // 确保每次发布的 Marker ID 不同
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING; // 使用文本标记
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 1.0; // 设置文本位置
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0; // 设置文本高度
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0; // 保持默认朝向
    marker.scale.x = 0.2; // 设置文本大小
    marker.scale.y = 0.2; // 设置文本大小
    marker.scale.z = 0.2; // 设置文本大小
    marker.color.a = 1.0; // 设置透明度
    if (currentStatus_ == Flat) {
        marker.color.r = 0.0; // 红色
        marker.color.g = 1.0; // 绿色
        marker.color.b = 0.0; // 蓝色
        marker.text = "Flat";
    } 
    else if (currentStatus_ == Up) {
        marker.color.r = 0.0; // 红色
        marker.color.g = 0.0; // 绿色
        marker.color.b = 1.0; // 蓝色
        marker.text = "Up";
    } 
    else if (currentStatus_ == Down) {
        marker.color.r = 1.0; // 红色
        marker.color.g = 1.0; // 绿色
        marker.color.b = 0.0; // 蓝色
        marker.text = "Down";
    } 
    else if (currentStatus_ == FlatToUp) {
        marker.color.r = 1.0; // 红色
        marker.color.g = 0.5; // 绿色
        marker.color.b = 0.5; // 蓝色
        marker.text = "FlatToUp";
    } 
    else if (currentStatus_ == FlatToDown) {
        marker.color.r = 1.0; // 红色
        marker.color.g = 0.5; // 绿色
        marker.color.b = 1.0; // 蓝色
        marker.text = "FlatToDown";
    }

    // marker.lifetime = rclcpp::Duration::from_seconds(0.1); // 设置标记的生命周期
    marker_pub_->publish(marker); // 发布 Marker 消息
}

void Podao_Detect::scan_callback_left(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{

    if (left_z_ == 0) {
        RCLCPP_WARN(this->get_logger(), "failed, the calibration_file is not exist");
        return;
    }
    Eigen::Matrix3d RotateMatrix;
    RotateMatrix = Eigen::AngleAxisd(left_Rz1_, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(left_Ry1_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(left_Rx1_, Eigen::Vector3d::UnitX());
    Eigen::Vector3d TransVector;

    TransVector << 0, left_y_, left_z_;

    TransMatrix_left.block(0, 0, 3, 3) = RotateMatrix;
    TransMatrix_left.block(0, 3, 3, 1) = TransVector;

    float check_range = 0.25;
    bool no_noise = check_data(scan_msg, check_range, 10);
    // 转换后的点云，debug 用
    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud_near(new PointCloud);

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

            if (p_base.z < 1.5 && p_base.x > 0.3) {
                cloud->points.push_back(p_base);
            }
            else if (p_base.z < 1.5 &&  p_base.x < 0.1 && p_base.x > -0.1) {
                cloud_near->points.push_back(p_base);
            }
        }
        // 更新角度
        angle += scan_msg->angle_increment;
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    PointCloud::Ptr cloud_all(new PointCloud);
    *cloud_all = *cloud;
    *cloud_all += *cloud_near;

    cloud_all->height = 1;
    cloud_all->width = cloud_all->points.size();
    cloud_all->is_dense = false;


    // sensor_msgs::msg::PointCloud2 ros_msgs_convert;
    // PCL2ROSMSg(cloud_all, ros_msgs_convert);
    // ros_msgs_convert.header = scan_msg->header;
    // ros_msgs_convert.header.frame_id = "base_link_map";
    // transform_pointcloud2_left_pub_->publish(ros_msgs_convert);

    podao_line_vector_left_.clear();
    // 拟合直线
    PointCloud::Ptr podao_line(new PointCloud);

    // 遍历 cloud_near 点云
    int point_count = 0;
    int ground_point_count = 0;
    for (const auto& point : cloud_near->points) {
        point_count++;
        if (point.z < 0.03) {
            ground_point_count++;
        }

    }

    if (point_count > 10 && ground_point_count < point_count * 0.3) {
        // sensor_msgs::msg::PointCloud2 ros_msgs_podao;
        // PCL2ROSMSg(podao_line, ros_msgs_podao);
        // ros_msgs_podao.header = scan_msg->header;
        // ros_msgs_podao.header.frame_id = "base_link_map";
        // podao_pointcloud2_left_pub_->publish(ros_msgs_podao);
        return;
    }
    get_podao_line(cloud, podao_line, podao_line_vector_left_);

    // sensor_msgs::msg::PointCloud2 ros_msgs_podao;
    // PCL2ROSMSg(podao_line, ros_msgs_podao);
    // ros_msgs_podao.header = scan_msg->header;
    // ros_msgs_podao.header.frame_id = "base_link_map";
    // podao_pointcloud2_left_pub_->publish(ros_msgs_podao);


    
}

void Podao_Detect::scan_callback_right(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{

    if (right_z_ == 0) {
        RCLCPP_ERROR(this->get_logger(), "failed, the calibration_file is not exist");
        return;
    }
    Eigen::Matrix3d RotateMatrix;
    RotateMatrix = Eigen::AngleAxisd(right_Rz1_, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(right_Ry1_, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(right_Rx1_, Eigen::Vector3d::UnitX());
    Eigen::Vector3d TransVector;
    TransVector << 0, right_y_, right_z_;

    TransMatrix_right.block(0, 0, 3, 3) = RotateMatrix;
    TransMatrix_right.block(0, 3, 3, 1) = TransVector;

    float check_range = 0.25;
    bool no_noise = check_data(scan_msg, check_range, 10);
    // 转换后的点云，debug 用
    PointCloud::Ptr cloud(new PointCloud);

    PointCloud::Ptr cloud_near(new PointCloud);

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

            if (p_base.z < 1.5 && p_base.x > 0.3) {
                cloud->points.push_back(p_base);
            }
            else if (p_base.z < 1.5 &&  p_base.x < 0.1 && p_base.x > -0.1) {
                cloud_near->points.push_back(p_base);
            }
        }
        // 更新角度
        angle += scan_msg->angle_increment;
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;

    PointCloud::Ptr cloud_all(new PointCloud);
    *cloud_all = *cloud;
    *cloud_all += *cloud_near;

    cloud_all->height = 1;
    cloud_all->width = cloud_all->points.size();
    cloud_all->is_dense = false;

    // sensor_msgs::msg::PointCloud2 ros_msgs1;
    // PCL2ROSMSg(cloud_all, ros_msgs1);
    // ros_msgs1.header = scan_msg->header;
    // ros_msgs1.header.frame_id = "base_link_map";
    // transform_pointcloud2_right_pub_->publish(ros_msgs1);

    podao_line_vector_right_.clear();

    PointCloud::Ptr podao_line(new PointCloud);

    // 解决狭窄区域内部误触发
    int point_count = 0;
    int ground_point_count = 0;
    for (const auto& point : cloud_near->points) {
        point_count++;
        if (point.z < 0.03) {
            ground_point_count++;
        }

    }

    if (point_count > 10 && ground_point_count < point_count * 0.3) {
        // sensor_msgs::msg::PointCloud2 ros_msgs_podao;
        // PCL2ROSMSg(podao_line, ros_msgs_podao);
        // ros_msgs_podao.header = scan_msg->header;
        // ros_msgs_podao.header.frame_id = "base_link_map";
        // podao_pointcloud2_right_pub_->publish(ros_msgs_podao);
        return;
    }
       
    get_podao_line(cloud, podao_line, podao_line_vector_right_);

    // sensor_msgs::msg::PointCloud2 ros_msgs_podao;
    // PCL2ROSMSg(podao_line, ros_msgs_podao);
    // ros_msgs_podao.header = scan_msg->header;
    // ros_msgs_podao.header.frame_id = "base_link_map";
    // podao_pointcloud2_right_pub_->publish(ros_msgs_podao);
    
}


void Podao_Detect::get_podao_line(PointCloud::Ptr cloud, PointCloud::Ptr podao_line, std::vector<PointCloud::Ptr> &pdao_line_vector)
{
    if (cloud->points.size() < 10) {
        RCLCPP_ERROR(logger_, "get_podao_line, cloud->points.size() < 10");
        return;
    }

    PointCloud::Ptr cloud_all(new PointCloud);
    *cloud_all = *cloud;

    auto starttime = std::chrono::system_clock::now();
    auto endtime = std::chrono::system_clock::now();
    auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(endtime - starttime).count();


    // // 排除正向高台影响
    // // 就近拟合
    // // 创建 SAC 分割对象并设置参数
    // SegLine seg_near;
    // seg_near.setThreshold(0.02); // 设定拟合的最大距离（阈值）
    // // 提取结果
    // pcl::ExtractIndices<pcl::PointXYZ> extract_near;
    // PointCloud::Ptr cloud_line_near(new PointCloud);
    // std::vector<std::vector<double>> positive_obstacle_line;

    // while (cloud->points.size() > 5) // 继续拟合直到剩余的点足够少
    // {
    //     // 存储提取出来的内点
    //     pcl::PointIndices::Ptr inlier_indices_near(new pcl::PointIndices);
    //     pcl::ModelCoefficients::Ptr coefficients_near(new pcl::ModelCoefficients);
    //     // 对点云进行分割
    //     seg_near.fitLineRANSAC(cloud, inlier_indices_near, coefficients_near);

    //     // 检查是否找到有效的模型
    //     if (inlier_indices_near->indices.size() < 5) {
    //         break;
    //     }

    //     // 提取内点并从点云中移除
    //     extract_near.setInputCloud(cloud);
    //     extract_near.setIndices(inlier_indices_near);

    //     if (coefficients_near->values[3] == 0 || coefficients_near->values[4] == 0) {
    //         positive_obstacle_line.push_back({ coefficients_near->values[0], coefficients_near->values[1], coefficients_near->values[2], coefficients_near->values[3], coefficients_near->values[4], coefficients_near->values[5] });
    //     }

    //     else if (std::max(abs(coefficients_near->values[5] / coefficients_near->values[3]), abs(coefficients_near->values[5] / coefficients_near->values[4])) > 1) {
    //         positive_obstacle_line.push_back({ coefficients_near->values[0], coefficients_near->values[1], coefficients_near->values[2], coefficients_near->values[3], coefficients_near->values[4], coefficients_near->values[5] });
    //     }
    //     extract_near.setNegative(true); // 设置负值，移除内点
    //     extract_near.filter(*cloud);
    // }


    // for (auto line : positive_obstacle_line) {
    //     pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
    //     fitLineRANSAC(cloud_all, 0.05, line, inlier_indices);
    //     // 提取内点并从点云中移除
    //     extract_near.setInputCloud(cloud_all);
    //     extract_near.setIndices(inlier_indices);

    //     extract_near.setNegative(true); // 设置负值，移除内点
    //     extract_near.filter(*cloud_all);
    // }


    // 拟合直线
    // 创建 SAC 分割对象并设置参数
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);  // 设置模型类型为直线模型
    seg.setMethodType(pcl::SAC_RANSAC);  // 使用 RANSAC 算法
    seg.setDistanceThreshold(0.04);  // 设定拟合的最大距离（阈值）
    // 提取结果
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    // 迭代拟合多条直线
    int line_id = 1;
    PointCloud::Ptr cloud_line(new PointCloud);

    while (cloud_all->points.size() > 15) // 继续拟合直到剩余的点足够少
    {
        // 存储提取出来的内点
        pcl::PointIndices::Ptr inlier_indices(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // 对点云进行分割
        seg.setInputCloud(cloud_all);
        seg.segment(*inlier_indices, *coefficients);

        // 检查是否找到有效的模型
        if (inlier_indices->indices.size() < 10) {
            break;
        }
        // // 可视化拟合的直线
        // std::cout << "Line " << line_id << ": " << std::endl;
        // std::cout << "point num " << inlier_indices->indices.size() << ": " << std::endl;
        // std::cout << "Coefficients: " <<
        //              coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " "
        //              << coefficients->values[3] << " " << coefficients->values[4] << " " << coefficients->values[5] << std::endl;
        line_id++;
        // visualization_msgs::msg::Marker line_marker;

        // line_marker.header.frame_id = "left_tilt_rotate"; // 你可以修改为 "base_link" 或其他坐标系
        // line_marker.header.stamp = this->now();
        // line_marker.ns = "lines";
        // line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        // line_marker.action = visualization_msgs::msg::Marker::ADD;
        // // 设置线的宽度
        // line_marker.scale.x = 0.01;

        // // 设置颜色（红色）
        // line_marker.color.r = 1.0;
        // line_marker.color.g = 1.0;
        // line_marker.color.b = 0.0;
        // line_marker.color.a = 1.0;

        // line_marker.id = line_id;

        // // 提取直线参数
        // double x0 = coefficients->values[0];
        // double y0 = coefficients->values[1];
        // double z0 = coefficients->values[2];
        // double dx = coefficients->values[3];
        // double dy = coefficients->values[4];
        // double dz = coefficients->values[5];

        // double t = 1.0; // 直线长度

        // // 计算直线起点和终点
        // geometry_msgs::msg::Point p1, p2;
        // p1.x = x0;
        // p1.y = y0;
        // p1.z = z0;

        // p2.x = x0 + t * dx;
        // p2.y = y0 + t * dy;
        // p2.z = z0 + t * dz;

        // // 添加到 Marker
        // line_marker.points.push_back(p1);
        // line_marker.points.push_back(p2);

        // marker_array.markers.push_back(line_marker);

        // 提取内点并从点云中移除
        extract.setInputCloud(cloud_all);
        extract.setIndices(inlier_indices);

        extract.setNegative(false); // 设置正值，提取直线点;
        extract.filter(*cloud_line);
        // 按角度聚类
        if (coefficients->values[3] != 0 && coefficients->values[4] != 0) {
            double slope = std::max(abs(coefficients->values[5] / coefficients->values[3]), abs(coefficients->values[5] / coefficients->values[4]));
            // std::cout << "slope: " << slope << std::endl;
            // 转换为角度
            double angle1 = atan2(coefficients->values[5], coefficients->values[3] ) * 180 / M_PI;
            angle1 = std::abs(angle1);
            if (angle1 > 90) {
                angle1  = abs(angle1 - 180);
            }
    
            double angle2 = atan2(coefficients->values[5], coefficients->values[4] ) * 180 / M_PI;
            angle2 = std::abs(angle2);
            if (angle2 > 90) {
                angle2  = abs(angle2 - 180);
            }
            RCLCPP_INFO(logger_, "angle1: %f, angle2: %f", angle1, angle2);
            double angle = std::max(abs(angle1), abs(angle2));
            if (angle >4 && angle < 20) {
                // 只保留坡道线
                // 聚类 
                get_min_cluster_filter(cloud_line, pdao_line_vector, 0.2, 5, 2);
                
            }
        }

        extract.setNegative(true); // 设置负值，移除内点
        extract.filter(*cloud_all);

    }

    std::sort(pdao_line_vector.begin(), pdao_line_vector.end(),
        [](PointCloud::Ptr a, PointCloud::Ptr b) {
            return a->points[0].x < b->points[0].x; // 按照 x 坐标排序
        });

    for (size_t i = 0; i < pdao_line_vector.size(); i++) {
        PointCloud::Ptr cloud_cluster = pdao_line_vector[i];
        *podao_line += *cloud_cluster;
    } 

    podao_line->height = 1;
    podao_line->width = podao_line->points.size();
    podao_line->is_dense = false;

}


void Podao_Detect::get_min_cluster_filter(PointCloud::Ptr input, std::vector<PointCloud::Ptr>& pdao_line_vector, double r, int min_points, int mode)
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

   
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        PointCloud::Ptr cloud_cluster(new PointCloud);
        // 只有一个点的情况

        float min_z, max_z, min_x, max_x, min_y, max_y;
        PointT min_z_point, max_z_point;
        min_z = input->points[*(it->indices.begin())].z;
        max_z = input->points[*(it->indices.begin())].z;

        min_x = input->points[*(it->indices.begin())].x;
        max_x = input->points[*(it->indices.begin())].x;

        min_y = input->points[*(it->indices.begin())].y;
        max_y = input->points[*(it->indices.begin())].y;

        min_z_point = input->points[*(it->indices.begin())];
        max_z_point = input->points[*(it->indices.begin())];
        
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++) {
            if (abs(input->points[*pit].z) < 0.01) {
                continue;
            }
            cloud_cluster->points.push_back(input->points[*pit]);
            min_z = std::min(min_z, input->points[*pit].z);
            max_z = std::max(max_z, input->points[*pit].z);

            min_x = std::min(min_x, input->points[*pit].x);
            max_x = std::max(max_x, input->points[*pit].x);

            min_y = std::min(min_y, input->points[*pit].y);
            max_y = std::max(max_y, input->points[*pit].y);
        }

        if (min_z < -0.3 || max_z > 0.3) {
           continue;
        }

        if (cloud_cluster->points.size() < 2) {
            continue; // 小于最小点数则跳过
        }

        std::sort(cloud_cluster->points.begin(), cloud_cluster->points.end(),
            [](const PointT& a, const PointT& b) {
                return a.x < b.x; // 按照 x 坐标排序
            });
        
        if (cloud_cluster->points[cloud_cluster->points.size()-1].x  - cloud_cluster->points[0].x < 0.3) {
            continue; // 小于最小点数则跳过
        }
        if (abs(cloud_cluster->points[cloud_cluster->points.size()-1].z) < 0.05) {
            continue; // 则跳过
        }
        if (abs(cloud_cluster->points[0].z) > 0.13) {
            continue; // 则跳过
        }
        if (cloud_cluster->points[0].x > 1) {
            continue;
        }

        if (cloud_cluster->points[0].x < 0.65 && abs(cloud_cluster->points[cloud_cluster->points.size()-1].x - cloud_cluster->points[0].x) < 0.5 ) {
            continue; // 小于最小点数则跳过
        }

        int is_valid_count = 0;
        PointT start_point = cloud_cluster->points[0];
        // 初始化end_point （0，0，0）
        PointT end_point(0, 0, 0);
        for (size_t i = 1; i < cloud_cluster->points.size(); i++) {
            // 计算每个点和start_point的距离
            double distance = std::sqrt(
                std::pow(cloud_cluster->points[i].x - start_point.x, 2) +
                std::pow(cloud_cluster->points[i].y - start_point.y, 2) +
                std::pow(cloud_cluster->points[i].z - start_point.z, 2)
            );
            if (distance > 0.15) {
                end_point = cloud_cluster->points[i];
                break; // 找到第一个距离大于0.15的点
            }
        }
        
        // 计算end_point 和 start_point 的斜率
        if (end_point.x != 0) {
            double det_x = abs(end_point.x - start_point.x);
            double det_y = abs(end_point.y - start_point.y);
            double det_z = abs(end_point.z - start_point.z);

            if (det_x == 0 || det_y == 0) {
                continue; // 如果斜率为无穷大，则跳过
            }
            else if (det_z / det_x > 0.26 || det_z / det_y > 0.26) {
                continue; // 如果斜率过大，则跳过
            }
        }

        for (const auto& point : cloud_cluster->points) {
            if (abs(point.z / (point.x)) > 0.26) {
                is_valid_count += 1;
            }
        }

        if (is_valid_count > 5) {
           continue;
        }

        pdao_line_vector.push_back(cloud_cluster);  
    }
}

void Podao_Detect::get_base_link_matrix(std::string flie_name,
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

void Podao_Detect::read_from_yaml(std::string flie_name) {   
    YAML::Node config = YAML::LoadFile(flie_name);
    if (!config) {
        RCLCPP_ERROR(logger_, "read_from_yaml infile: %ld ms %s", errno, flie_name.c_str());
        return;
    }

   if (config["pitch_base"]) {
        pitch_base_ = config["pitch_base"].as<double>();
        RCLCPP_INFO(this->get_logger(), "pitch_base_: %f", pitch_base_);
   }
}

bool Podao_Detect::check_data(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, float check_range, int min_num)
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

// void Podao_Detect::scan_callback_left_Hori(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
// {
//     if (left_lidar_hori_x == 0 || left_lidar_hori_y == 0) {
//         try {
//             geometry_msgs::msg::TransformStamped transform_left = tf_buffer_->lookupTransform(
//                 "base_link",  // 目标坐标系
//                 scan_msg->header.frame_id,  // 源坐标系
//                 scan_msg->header.stamp);  // 时间对齐
//             RCLCPP_INFO(this->get_logger(), "transform-left x  %f", transform_left.transform.translation.x);
//             RCLCPP_INFO(this->get_logger(), "transform-left y  %f", transform_left.transform.translation.y);
//             left_lidar_hori_x = transform_left.transform.translation.x;
//             left_lidar_hori_y = transform_left.transform.translation.y;

//             translation_left_hori = tf2::Vector3(transform_left.transform.translation.x, transform_left.transform.translation.y, transform_left.transform.translation.z);
//             rotation_matrix_left_hori = tf2::Matrix3x3(tf2::Quaternion(transform_left.transform.rotation.x, transform_left.transform.rotation.y, transform_left.transform.rotation.z, transform_left.transform.rotation.w));

//         }
//         catch (const tf2::TransformException &ex)
//         {
//             // RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
//         }
//         return;
//     }

//     cloud_all_left_hori->points.clear();

//     float check_range = 0.20;
//     bool no_noise = check_data(scan_msg, check_range, 5);

//     float angle = scan_msg->angle_min;  // 当前角度（从左到右扫描
//     for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
//     {
//         float range = scan_msg->ranges[i];  // 获取当前点的距离
//         PointT p;
//         // 如果距离有效（大于最小距离且小于最大距离）
//         if (range >= scan_msg->range_min && range <= scan_msg->range_max)
//         { 
//             // 如果存在噪点，消除距离太小值
//             if (!no_noise && range < check_range) {
//                 angle += scan_msg->angle_increment;
//                 continue;
//             }
//             // 转换为笛卡尔坐标（x, y）
//             // p.x = range * std::cos(angle);
//             // p.y = range * std::sin(angle);

//             tf2::Vector3 point_temp_origin(range * std::cos(angle), range * std::sin(angle), 0);
//             tf2::Vector3 point_temp = rotation_matrix_left_hori * point_temp_origin + translation_left_hori;
            
//             if (point_temp[0] > 0.8) {
//                 // 存储坐标点
//                 p.x = point_temp[0];
//                 p.y = point_temp[1];
//                 p.z = point_temp[2];
//                 cloud_all_left_hori->points.push_back(p);

//             }    
//         }
//         // 更新角度
//         angle += scan_msg->angle_increment;
//     }

//     cloud_all_left_hori->height = 1;
//     cloud_all_left_hori->width = cloud_all_left_hori->points.size();
//     cloud_all_left_hori->is_dense = false;
// }


// void Podao_Detect::scan_callback_right_Hori(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
// {

//     if (right_lidar_hori_x == 0 || right_lidar_hori_y == 0) {
//         try {
//             geometry_msgs::msg::TransformStamped transform_right = tf_buffer_->lookupTransform(
//                 "base_link",  // 目标坐标系
//                 scan_msg->header.frame_id,  // 源坐标系
//                 scan_msg->header.stamp);  // 时间对齐
//             RCLCPP_INFO(this->get_logger(), "transform-right x  %f", transform_right.transform.translation.x);
//             RCLCPP_INFO(this->get_logger(), "transform-right y  %f", transform_right.transform.translation.y);
//             right_lidar_hori_x = transform_right.transform.translation.x;
//             right_lidar_hori_y = transform_right.transform.translation.y;

//             translation_right_hori = tf2::Vector3(transform_right.transform.translation.x, transform_right.transform.translation.y, transform_right.transform.translation.z);
//             rotation_matrix_right_hori = tf2::Matrix3x3(tf2::Quaternion(transform_right.transform.rotation.x, transform_right.transform.rotation.y, transform_right.transform.rotation.z, transform_right.transform.rotation.w));

//         }
//         catch (const tf2::TransformException &ex)
//         {
//             // RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
//         }
//         return;
//     }

//     cloud_all_right_hori->points.clear();
//     float check_range = 0.20;
//     bool no_noise = check_data(scan_msg, check_range, 5);

//     float angle = scan_msg->angle_min;  // 当前角度（从左到右扫描
//     for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
//     {
//         float range = scan_msg->ranges[i];  // 获取当前点的距离
//         PointT p;
//         // 如果距离有效（大于最小距离且小于最大距离）
//         if (range >= scan_msg->range_min && range <= scan_msg->range_max)
//         { 
//             // 如果存在噪点，消除距离太小值
//             if (!no_noise && range < check_range) {
//                 angle += scan_msg->angle_increment;
//                 continue;
//             }
//             // 转换为笛卡尔坐标（x, y）
//             // p.x = range * std::cos(angle);
//             // p.y = range * std::sin(angle);
           
//             tf2::Vector3 point_temp_origin(range * std::cos(angle), range * std::sin(angle), 0);
//             tf2::Vector3 point_temp = rotation_matrix_right_hori * point_temp_origin + translation_right_hori;
            
//             if (point_temp[0] > 0.8) {
//                 // 存储坐标点
//                 p.x = point_temp[0];
//                 p.y = point_temp[1];
//                 p.z = point_temp[2];
//                 cloud_all_right_hori->points.push_back(p);

//             }    
        
//         }
//         // 更新角度
//         angle += scan_msg->angle_increment;
//     }

//     cloud_all_right_hori->height = 1;
//     cloud_all_right_hori->width = cloud_all_right_hori->points.size();
//     cloud_all_right_hori->is_dense = false;

// }



// void Podao_Detect::front_obstacle_Callback()
// {
//     PointCloud::Ptr cloud_all(new PointCloud);
//     *cloud_all = *cloud_all_left_hori + *cloud_all_right_hori;
//     if (cloud_all->points.size() < 10) {
//         is_front_obstacle_ = false;
//         std::cout << "is_front_obstacle_: " << is_front_obstacle_ << std::endl;
//         return;
//     }
//     std::vector<Point> rectangle_points;
//     Point p1, p2, p3, p4, p_middle;
//     p1 = {1.0, 0.3};
//     p2 = {1.0, -0.3};
//     p3 = {2.0, 0.3};
//     p4 = {2.0, -0.3};
//     rectangle_points.push_back(p1);
//     rectangle_points.push_back(p2);
//     rectangle_points.push_back(p3);
//     rectangle_points.push_back(p4);

//     p_middle = {p1.x+p2.x+p3.x+p4.x, p1.y+p2.y+p3.y+p4.y};
//     p_middle.x = p_middle.x / 4;
//     p_middle.y = p_middle.y / 4;
//     sortPointsCounterClockwise(rectangle_points, p_middle);

//     // 判断 是否有点落在矩形框内
//     bool is_in_rect = false;
//     PointCloud::Ptr cloud_in(new PointCloud);
//     isPointInRotatedRectangle(cloud_all, rectangle_points, cloud_in, is_in_rect);
   
//     if (is_in_rect) {
//         is_front_obstacle_ = true;
//     }
//     else {
//         is_front_obstacle_ = false;
//     }

//     std::cout << "is_front_obstacle_: " << is_front_obstacle_ << std::endl;
// }


