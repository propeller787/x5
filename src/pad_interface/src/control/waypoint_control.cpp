#include "waypoint_control.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


/*
imu：
1、西南路口
    朝西 2.9957
    朝北 1.2101
5号楼平行
    朝东 -0.5014
    朝西 2.65
    朝南 -1.9357
    朝北 1.25
公园小路1
    前半段: 1.4123
    后半段：0.7613
*/

#define IMU_OFFSET_ -1.09
#define DEF_DISTANCE 2.6

using std::placeholders::_1;
using namespace service::control;

constexpr double R = 6371000.0;
// 将角度转换为弧度
double toRadians(double degree) {
    return degree * M_PI / 180.0;
}

// 将弧度转换为角度
double toDegrees(double radian) {
    return radian * 180.0 / M_PI;
}


// 计算两个经纬度之间的距离
double haversine(double lat1, double lon1, double lat2, double lon2) {
    // 将纬度和经度从度数转换为弧度
    double phi1 = toRadians(lat1);
    double phi2 = toRadians(lat2);
    double deltaPhi = toRadians(lat2 - lat1);
    double deltaLambda = toRadians(lon2 - lon1);

    // Haversine公式
    double a = std::sin(deltaPhi / 2.0) * std::sin(deltaPhi / 2.0) +
               std::cos(phi1) * std::cos(phi2) *
               std::sin(deltaLambda / 2.0) * std::sin(deltaLambda / 2.0);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    // 计算距离
    double distance = R * c;
    return distance;
}

double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
    // 将纬度和经度从度数转换为弧度
    double phi1 = toRadians(lat1);
    double phi2 = toRadians(lat2);
    double deltaLambda = toRadians(lon2 - lon1);

    // 计算航向角
    double y = sin(deltaLambda) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(deltaLambda);
    double bearing = atan2(y, x);
    // printf("%f %f %f %f ++ %0.3f\n", lat1, lon1, lat2, lon2, bearing);
    return bearing;

    // 将航向角转换为范围[0, 360]度
    double bearing_degrees = toDegrees(bearing);
    bearing_degrees = fmod((bearing_degrees + 360.0), 360.0); // 确保在0到360度之间

    return bearing_degrees; // 返回的是度数
}

// 使用哈弗辛公式计算两点之间的距离
void calcHaversine(
    double lat1, double lon1,
    double lat2, double lon2, double& distance1, double& yaw1) {
    distance1  = haversine(lat1, lon1, lat2, lon2);
    yaw1 = calculateBearing(lat1, lon1, lat2, lon2);
}

WayPoint parse_line(const std::string& str)
{
    WayPoint out;
    std::istringstream iss(str);
    std::string token;

    // 使用逗号分隔符读取前两个浮点数
    std::getline(iss, token, ',');
    out.lng = std::stof(token);

    std::getline(iss, token, ',');
    out.lat = std::stof(token);

    // 剩余的字符串包含最后的三个数，它们之间由短横线分隔
    std::string remaining = token; // 这里已经有剩余的字符串

    // 使用短横线作为分隔符分割剩余的字符串
    std::getline(iss, token, '-');
    out.target_dur = std::stof(token);
    std::getline(iss, token, '-');
    out.max_speed = std::stof(token);
    std::getline(iss, token, '-');
    out.lat_offset = std::stof(token);
    std::getline(iss, token, '-');
    out.lng_offset = std::stof(token);

    std::cout << "Longitude: " << out.lng << std::endl;
    std::cout << "Latitude: " << out.lat << std::endl;
    if (out.target_dur!=0 || out.max_speed!=0 || out.max_speed!=0 || out.max_speed!=0) {
        std::cout << "target_dur : " << out.target_dur;
        std::cout << " max_speed : " << out.max_speed;
        std::cout << " lat_offset : " << out.lat_offset;
        std::cout << " lng_offset : " << out.lng_offset << std::endl;
    }
    return out;
}

std::vector<WayPoint> readGpsData(const std::string& filename) {
/*
116.122421,39.893105,90.67
116.122420,39.893105,90.64
116.122420,39.893105,90.65
*/
    std::vector<WayPoint> waypoint;
    std::ifstream file(filename);
    std::string line;
    if (file.is_open()) {
        while (std::getline(file, line)) {
            if (line.length() > 10) {
                WayPoint p = parse_line(line);
                waypoint.push_back(p);
            }
        }
        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
    }

    return waypoint;
}


std::vector<Gps84> interpolate(const Gps84& p1, const Gps84& p2, double resolution) {
    std::vector<Gps84> result;

    // 计算两点之间的距离
    double distance = haversine(p1.lat, p1.lng, p2.lat, p2.lng);
    
    // 如果距离小于分辨率，直接返回
    if (distance < resolution) {
        result.push_back(p2);
        return result;
    }

    // 计算需要插值的点数
    int num_points = static_cast<int>(distance / resolution);

    // 插值生成新点
    for (int i = 1; i <= num_points; ++i) {
        double fraction = (double)i / num_points;
        double lat = p1.lat + fraction * (p2.lat - p1.lat);
        double lon = p1.lng + fraction * (p2.lng - p1.lng);
        result.push_back(Gps84(lat, lon));
    }

    result.push_back(p2); // 将p2作为最后一个点加入

    return result;
}

void completion_gps_list(const std::vector<WayPoint>& input_points, std::vector<WayPoint>& out_points)
{
    double resolution = 0.2;
    for (size_t i = 0; i < input_points.size() - 1; ++i) {
        Gps84 p1(input_points[i].lat, input_points[i].lng);
        Gps84 p2(input_points[i+1].lat, input_points[i+1].lng);
        // 插值生成新点
        std::vector<Gps84> new_points = interpolate(p1, p2, resolution);
        // 添加新生成的点到总的轨迹点集合中
        out_points.push_back(input_points[i]); // 加入第一个点
        for (auto& ch : new_points) {
            WayPoint p;
            p.lat = ch.lat;
            p.lng = ch.lng;
            out_points.push_back(p);
        }
    }
    // // 打印补充后的GPS轨迹点
    // std::cout << "补充后的GPS轨迹点 (0.2米一个点):" << std::endl;
    // for (const auto& point : out_points) {
    //     std::cout << "纬度: " << point.lat << ", 经度: " << point.lng << std::endl;
    // }
    // sleep(10);
}

// void test()
// {
//     std::vector<Gps84> gps_points = {
//         Gps84(39.9042, 116.4074),  // 北京
//         Gps84(39.9048, 116.4080),  // 点2 (相距较近)
//         Gps84(39.9055, 116.4090)   // 点3
//     };
//     std::vector<Gps84> fixed_gps_points;
//     completion_gps_list(gps_points, fixed_gps_points);
// }


//imu_yaw_offset_ 10.7 21:33 -1.206016
WaypointControl::WaypointControl(): Control("waypoint"), send_interval_ms_(500), imu_yaw_nofix_(0.0), imu_yaw_offset_(IMU_OFFSET_), carGPS_(0.0, 0.0)
{
    auto global = service::Global::getInstance();
    imu_yaw_offset_ = global->node_->declare_parameter<double>("imu_offset", imu_yaw_offset_);
    const char* myEnvVar = std::getenv("imu_offset");
    if (myEnvVar != nullptr) {
        imu_yaw_offset_ = std::stod(myEnvVar);
        std::cout << "The value of imu_offset is: " << myEnvVar << std::endl;
    }
    target_dur_ = global->node_->declare_parameter<double>("target_dur", DEF_DISTANCE);
    lat_offset_ = global->node_->declare_parameter<double>("lat_offset", 0.0);
    lng_offset_ = global->node_->declare_parameter<double>("lng_offset", 0.0);
    max_speed_ = global->node_->declare_parameter<double>("max_speed", 0.8);
    parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(global->node_.get(), "/vlago_ctrl");
    parameter_callback_handle_ = global->node_->add_on_set_parameters_callback(std::bind(&WaypointControl::onSetParam, this, std::placeholders::_1));
    gps_sub_ = global->node_->create_subscription<sensor_msgs::msg::NavSatFix>(gpsTopic, 10, std::bind(&WaypointControl::onGpsResult, this, _1));
    imu_sub_ = global->node_->create_subscription<sensor_msgs::msg::Imu>(std::string("/imu"), 10, std::bind(&WaypointControl::onImuResult, this, _1));
    thread_ = std::make_unique<std::thread>([this](){
        std::cout << "wait 5 sec" << std::endl;
        sleep(5);
        loopSendTarget();
    });
}

rcl_interfaces::msg::SetParametersResult WaypointControl::onSetParam(const std::vector<rclcpp::Parameter> & parameters)
{
    std::cout << "param onSetParam " << std::endl;
    for (const auto &parameter : parameters) {
        if (parameter.get_name() == "imu_offset") {
            imu_yaw_offset_ = parameter.as_double();
            std::cout << "param change imu_offset: " << imu_yaw_offset_ << std::endl;
        } else if (parameter.get_name() == "lat_offset") {
            lat_offset_ = parameter.as_double();
            std::cout << "param change lat_offset: " << lat_offset_ << std::endl;
        } else if (parameter.get_name() == "lng_offset") {
            lng_offset_ = parameter.as_double();
            std::cout << "param change lng_offset: " << lng_offset_ << std::endl;
        } else if (parameter.get_name() == "max_speed") {
            double max_speed = parameter.as_double();
            if (abs(max_speed - max_speed_) > 0.01) {
                max_speed_ = max_speed;
                changeSpeed(max_speed_);
            }
        } else if (parameter.get_name() == "target_dur") {
            target_dur_ = parameter.as_double();
            std::cout << "param change target_dur: " << target_dur_ << std::endl;
        }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void WaypointControl::onGpsResult(const sensor_msgs::msg::NavSatFix msg)
{
    //lat 1米 0.0000089932
    //lon 1米  0.0000117215
    static uint32_t count = 0;
    if (count % 50 == 1) {
        printf("onGpsResult 2 lat:%0.7f lng:%0.7f offset lat:%0.3f lng:%0.3f\n", msg.latitude, msg.longitude, lat_offset_, lng_offset_);
    }
    Gps84 gps_data(msg.latitude + (0.0000089932 * lat_offset_), msg.longitude + (0.0000117215*lng_offset_));
    onCarGps(&gps_data, imu_yaw_nofix_);
}

void WaypointControl::onImuResult(const sensor_msgs::msg::Imu msg)
{
// #if 0
//     // 获取四元数
//     auto orientation = msg.orientation;

//     // 将四元数转换为RPY角（roll, pitch, yaw）
//     double roll, pitch, yaw;
//     tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

//     // 计算朝东方向的yaw
//     // 假设imu坐标系的z轴指向北，x轴指向东
//     // yaw角是从北方向顺时针测量的，我们需要转换为从东方向测量
//     yaw = M_PI / 2 - yaw;

//     // 确保yaw在-pi到pi之间
//     yaw = std::fmod(yaw + M_PI, 2 * M_PI) - M_PI;
//     imu_yaw_nofix_ = yaw;

//     // 输出yaw角
//     std::cout << "Yaw angle (radians): %f" << yaw << std::endl;
// #else
    // 从IMU消息中提取四元数
    tf2::Quaternion quat(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);

    // 将四元数转换为欧拉角
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // 假设朝东的方向为yaw = 0弧度，调整yaw到以东方为参考的方向
    // 将yaw转换为范围[-pi, pi]
    if (yaw < -M_PI)
        yaw += 2.0 * M_PI;
    else if (yaw > M_PI)
        yaw -= 2.0 * M_PI;
    static uint32_t count = 0;
    if (count % 1000 == 1) {
        printf("imu nofix yaw:%f fixed:%f \n", yaw, yaw + imu_yaw_offset_);
    }
    count++;
    
    imu_yaw_nofix_ = yaw;
// #endif
}

void WaypointControl::get_parameter(std::shared_ptr<rclcpp::AsyncParametersClient> client, const std::string & param_name)
{
#if 0
    auto future = client->get_parameters(param_name);

    future.then([this, param_name](rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedFutureWithRequestAndResponse result) {
        if (result.get().second->values.empty()) {
            printf("Failed to get parameter: %s", param_name.c_str());
            return;
        }

        double max_speed = result.get().second->values[0].double_value;
        printf("Current Max speed: %.2f", max_speed);
    });
#endif
}

void WaypointControl::set_parameter(std::shared_ptr<rclcpp::AsyncParametersClient> client, const std::string & param_name, double value)
{
#if 0
    auto parameter = rclcpp::Parameter(param_name, value);
    auto future = client->set_parameters({parameter});

    future.then([this, param_name, value](rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFutureWithRequestAndResponse result) {
        if (result->second->results[0].successful) {
            printf("Successfully set %s to %.2f", param_name.c_str(), value);
        } else {
            printf("Failed to set parameter: %s", param_name.c_str());
        }
    });
#endif
}

bool WaypointControl::changeSpeed(double speed)
{
#if 0
    while (!parameters_client_->wait_for_service(std::chrono::seconds(1))) {
        printf("Waiting for /vlago_ctrl parameter service to be available...");
    }
    get_parameter(parameters_client_, "max_speed");
    set_parameter(parameters_client_, "max_speed", speed);
#else
    if (thread_param_.get()) {
        thread_param_.reset();
    }
    thread_param_ = std::make_unique<std::thread>([this, speed](){
        char cmd[256];
        sprintf(cmd, "ros2 param set /vlago_ctrl max_speed %f", speed);
        system(cmd);
        // loopSendTarget();
    });
    thread_param_->detach();
    return true;
#endif
}

void WaypointControl::loopSendTarget()
{
    auto global = service::Global::getInstance();
    std::vector<WayPoint> read_list = readGpsData("data.txt");
    std::vector<WayPoint> target_list;
    completion_gps_list(read_list, target_list);
    // for(auto target : target_list) {
    //     printf("lat: %0.7f lng: %0.7f\n", target.lat, target.lng);
    // }

    while (carGPS_.lat <= 0.0) {
	    usleep(10*1000);
    }
    Gps84 last_gps(0.0, 0.0);
    for (const struct WayPoint& waypoint : target_list) {
        if (waypoint.target_dur > 0.1) {
            target_dur_ = waypoint.target_dur;
            std::cout << "change ---------- target_dur : " << target_dur_ << std::endl;
            global->node_->set_parameters({rclcpp::Parameter("target_dur", target_dur_)});
        }
        if (waypoint.max_speed != 0.0) {
            if (abs(waypoint.max_speed - max_speed_) > 0.01) {
                std::cout << "change 1 ------------------------ max_speed : " << max_speed_ << std::endl;
                max_speed_ = waypoint.max_speed;
                changeSpeed(max_speed_);
                global->node_->set_parameters({rclcpp::Parameter("max_speed", max_speed_)});
                std::cout << "change 2 ----------------------- max_speed : " << max_speed_ << std::endl;
            }
        }
        if (waypoint.lat_offset != 0.0) {
            lat_offset_ = waypoint.lat_offset;
            std::cout << "change ---------- lat_offset : " << lat_offset_ << std::endl;
            global->node_->set_parameters({rclcpp::Parameter("lat_offset", lat_offset_)});
        }
        if (waypoint.lat_offset != 0.0) {
            lng_offset_ = waypoint.lng_offset;
            std::cout << "change ---------- lng_offset : " << lng_offset_ << std::endl;
            global->node_->set_parameters({rclcpp::Parameter("lng_offset", lng_offset_)});
        }
        Gps84 target(waypoint.lat, waypoint.lng);
        double distance = 0.0;
        double diffEastYaw = 0.0;
        if (last_gps.lat == 0.0) {
            last_gps = target;
            last_gps.lat = target.lat;
            last_gps.lng = target.lng;
            continue;
        }
        calcDiffGps(&carGPS_, &target, distance, diffEastYaw);
        while (distance > 5.0) {
            printf("waitGps  distance:%f target_dur : %f\n", distance, target_dur_);
            usleep(300*1000);
            calcDiffGps(&carGPS_, &target, distance, diffEastYaw);
        }
        if (distance < 0.6) {
            last_gps.lat = target.lat;
            last_gps.lng = target.lng;
            std::cout << "ignore lat : " << target.lat << " - lng: " << target.lng << std::endl;
            continue;
        }
        if (!waitGps(&target, &last_gps)) {
            std::cout << "waitGps exit : " << std::endl;
            continue;
        }
        last_gps.lat = target.lat;
        last_gps.lng = target.lng;
        // std::cout << "proc lat : " << target.lat << " - lng: " << target.lng << std::endl;
        handleTarget(&target);
    }
}

bool WaypointControl::waitGps(Gps84* target, Gps84* last_target)
{
    double distance = 0.0;
    double diffEastYaw = 0.0;
    uint32_t count = 0;
    while (true) {
        calcDiffGps(&carGPS_, target, distance, diffEastYaw);
        if (count % 10 == 6) {
            printf("carGPS %f %f\n", carGPS_.lat, carGPS_.lng);
            printf("waitGps  distance:%f target_dur : %f\n", distance, target_dur_);
        }
        count ++;
        if (distance < target_dur_) {
            std::cout << "waitGps true distance : " << distance << " target_dur : " << target_dur_ << std::endl;
            if (count < 2) {
                usleep(100*1000);
            }
            return true;
        }
        // if (distance > 100) {
        //     std::cout << "waitGps error distance : " << distance << std::endl;
        //     return false;
        // }
        usleep(200*1000);
        if (last_target->lat > 0.0 && count % 4 == 2) {
            calcDiffGps(&carGPS_, last_target, distance, diffEastYaw);
            if (distance > 1.2) {
                handleTarget(last_target);
            }
        }
    }
}

void WaypointControl::handleTarget(Gps84* target)
{
    double distance = 0.0;
    double diffEastYaw = 0.0;
    calcDiffGps(target, &carGPS_, distance, diffEastYaw);
    // printf("calcDiffGps %f %f %f %f -----distance:%f diffEastYaw:%0.3f\n", carGPS_.lat, carGPS_.lng, target->lat, target->lng, distance, diffEastYaw);
    double imuFixedYaw = imu_yaw_;
    double baselinkYaw = 0.0;
    calcBaseLinkYaw(imuFixedYaw, diffEastYaw, baselinkYaw);

    BaseLinkPose baselinkPose;
    calcBaseLinkPose(baselinkYaw, distance, baselinkPose);
    sendBasePose(&baselinkPose);
}

void WaypointControl::onCarGps(struct Gps84* carGPS, double imu_yaw_nofix)
{
    carGPS_ = *carGPS;
    // printf("onCarGps carGPS_ 1 lat:%f lng:%f\n", carGPS_.lat, carGPS_.lng);
    imu_yaw_ = fixImuYaw(imu_yaw_nofix);
}

double WaypointControl::fixImuYaw(double imu_yaw_nofix)
{
    return imu_yaw_nofix + imu_yaw_offset_;
}

void WaypointControl::calcDiffGps(Gps84* car, Gps84* target, double& distance, double& diffEastYaw)
{
    calcHaversine(target->lat,  target->lng, car->lat, car->lng, distance, diffEastYaw);
}

void WaypointControl::calcBaseLinkYaw(double imuFixedYaw, double diffEastYaw, double& baselinkYaw)
{
    baselinkYaw = imuFixedYaw + diffEastYaw;
    if (baselinkYaw < -M_PI) {
        baselinkYaw += 2*M_PI;
    } else if (baselinkYaw >= M_PI) {
        baselinkYaw -= 2*M_PI;
    }
    // printf("calcBaseLinkYaw imu:%f east:%f baselink:%f\n", imuFixedYaw, diffEastYaw, baselinkYaw);
}

void WaypointControl::calcBaseLinkPose(double baselinkYaw, double distance, BaseLinkPose& baselinkPose)
{
    baselinkPose.x = 0.0;
    baselinkPose.y = 0.0;
    // printf("calcBaseLinkPose 1 baselinkYaw:%f distance:%f\n", baselinkYaw, distance);
    baselinkPose.y += distance * cos(baselinkYaw);
    baselinkPose.x += distance * sin(baselinkYaw);
    // printf("calcBaseLinkPose 2 x:%f y:%f\n", baselinkPose.x, baselinkPose.y);
    baselinkPose.yaw = baselinkYaw;
}

bool WaypointControl::sendBasePose(BaseLinkPose* baselinkPose)
{
    // TODO tf树转化
    auto global = service::Global::getInstance();
    geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
    pointSta->header.stamp = rclcpp::Time();
    pointSta->point.x = baselinkPose->x;
    pointSta->point.y = baselinkPose->y;
    printf("sendBasePose x:%f y:%f yaw:%f\n", baselinkPose->x, baselinkPose->y, baselinkPose->yaw);
    global->send_playback_pose(pointSta);
    return true;
}


