#ifndef __PERCEPTION_UTILS_H__
#define __PERCEPTION_UTILS_H__

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/msg/image.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <filesystem>
#include <string>
#include <cstring>
#include <net/if.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <ifaddrs.h>
#include <arpa/inet.h>


// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 子节点基类，所有子节点继承它
class BaseNode : public rclcpp::Node
{
public:
    BaseNode(const std::string &node_name) : Node(node_name), is_active_(false)
    {
    }

    void activate()
    {
        is_active_ = true;
        RCLCPP_INFO(this->get_logger(), "%s activated", this->get_name());
    }

    void deactivate()
    {
        is_active_ = false;
        RCLCPP_INFO(this->get_logger(), "%s deactivated", this->get_name());
    }


    bool is_active_;
};

struct Point {
    double x, y;

    // 默认构造函数
    Point() : x(0), y(0) {}

    // 带参构造函数
    Point(double xVal, double yVal) : x(xVal), y(yVal) {}

    // 拷贝构造函数
    Point(const Point &other) : x(other.x), y(other.y) {}

    // 重载赋值运算符
    Point& operator=(const Point &other) {
        // 检查自赋值
        if (this != &other) {
            x = other.x;
            y = other.y;
        }
        return *this;
    }

    Point operator+(const Point& other) const {
        return Point(x + other.x, y + other.y);
    }
    
    Point operator-(const Point& other) const {
        return Point(x - other.x, y - other.y);
    }
    
    Point operator*(double scalar) const {
        return Point(x * scalar, y * scalar);
    }

    Point operator/(double scalar) const {
        return Point(x / scalar, y / scalar);
    }

};

void initRosMsgFields(std::vector<sensor_msgs::msg::PointField>& fields);
void PCL2ROSMSg(const PointCloud::Ptr cloud_ptr, sensor_msgs::msg::PointCloud2& ros_msgs);
std::string getAbsolutePath(const std::string& relativePath);
std::string getProcessPath(void);


std::string getLocalIPAddress();
std::string get_mac_address(const std::string &interface_name);


#endif //__PERCEPTION_UTILS_H__