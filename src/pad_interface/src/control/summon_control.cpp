#include "summon_control.h"
using namespace service::control;

double haversineDistance(const sensor_msgs::msg::NavSatFix& a, const sensor_msgs::msg::NavSatFix& b)
{
    // 使用 Haversine 公式计算两个 GPS 点之间的距离
    const double R = 6371000.0; // 地球半径（米）
    double lat1 = a.latitude * M_PI / 180.0;
    double lon1 = a.longitude * M_PI / 180.0;
    double lat2 = b.latitude * M_PI / 180.0;
    double lon2 = b.longitude * M_PI / 180.0;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    double a_val = std::sin(dlat / 2) * std::sin(dlat / 2) + std::cos(lat1) * std::cos(lat2) * std::sin(dlon / 2) * std::sin(dlon / 2);

    double c = 2 * std::atan2(std::sqrt(a_val), std::sqrt(1 - a_val));
    return R * c;
}

SummonControl::SummonControl()
    : Control("SummonControl")
{
    using namespace std::placeholders;
    auto global = service::Global::getInstance();
    car_summon_ = global->node_->create_service<interface::srv::Summon>(summonService, std::bind(&SummonControl::handleSummon, this, _1, _2));
    car_summon_status_ = global->node_->create_service<interface::srv::SummonStatus>(summonStatusService, std::bind(&SummonControl::handleSummonStatus, this, _1, _2));
   // loadGPSPoints("gps_point_list.txt");
}

void SummonControl::handleSummon(const std::shared_ptr<interface::srv::Summon::Request> request,
    std::shared_ptr<interface::srv::Summon::Response> response)
{
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }
    auto global = service::Global::getInstance();
    double enux = 0.0, enuy = 0.0;
    double lat = (double)request->lat;
    double lon = (double)request->lon;
    bool northp = false;
    {
        sensor_msgs::msg::NavSatFix current_point, nearest_point;
        current_point.latitude = lat;
        current_point.longitude = lon;
        nearest_point = get_nearest_point(current_point);
        printf("match latitude:%0.8f - %0.8f latitude:%0.8f - %0.8f \n",
            current_point.latitude, nearest_point.latitude,
            current_point.longitude, nearest_point.longitude);

        double distance = haversineDistance(current_point, nearest_point);
        printf("distance:%0.3f latitude:%0.8f latitude:%0.8f \n", distance, nearest_point.latitude, nearest_point.longitude);
        lat = nearest_point.latitude;
        lon = nearest_point.longitude;
    }
    // GeographicLib::UTMUPS::Forward(lat, lon, utm_zone_, northp, enux, enuy);
    std::cout << std::fixed << std::setprecision(7) << " lat " << lat << " lon " << lon << "  " << request->precision << std::endl;
    std::cout << std::fixed << std::setprecision(7) << " Summon utm x " << enux << " y " << enuy << "value x " << enux - utm_x_ << " value y " << enuy - utm_y_ << std::endl;
    geometry_msgs::msg::PointStamped::SharedPtr pointSta = std::make_shared<geometry_msgs::msg::PointStamped>();
    pointSta->header.stamp = rclcpp::Time();
    pointSta->point.x = enux - utm_x_;
    pointSta->point.y = enuy - utm_y_;
    pointSta->header.frame_id = "map";
    global->send_goal_summon(pointSta);
    response->set__code(0);
}
void SummonControl::handleSummonStatus(const std::shared_ptr<interface::srv::SummonStatus::Request> request,
    std::shared_ptr<interface::srv::SummonStatus::Response> response)
{
    if (id_ != request->id) {
        response->set__code(-1);
        return;
    }

    response->set__code(0);
    response->set__status(3);
}

void SummonControl::loadGPSPoints(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Failed to open file:" << filename << std::endl;
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string token;
        sensor_msgs::msg::NavSatFix point;
        if (line.length() < 10) {
            continue;
        }

        if (std::getline(iss, token, ',')) {
            point.longitude = std::stod(token);
        }
        if (std::getline(iss, token, ',')) {
            point.latitude = std::stod(token);
        }
        if (std::getline(iss, token, ',')) {
            point.altitude = std::stod(token);
        }
        printf("latitude:%0.7f latitude:%0.7f \n", point.latitude, point.longitude);

        gps_points_.push_back(point);
    }
    file.close();
}

sensor_msgs::msg::NavSatFix SummonControl::get_nearest_point(const sensor_msgs::msg::NavSatFix& current_point)
{
    sensor_msgs::msg::NavSatFix nearest_point;
    nearest_point.latitude = current_point.latitude;
    nearest_point.longitude = current_point.longitude;
    if (gps_points_.empty()) {
        std::cout << "GPS point list is empty." << std::endl;
        return nearest_point;
    }
    double min_distance = std::numeric_limits<double>::max();

    for (const auto& point : gps_points_) {
        double distance = haversineDistance(current_point, point);
        if (distance < min_distance) {
            min_distance = distance;
            nearest_point = point;
        }
    }
    return nearest_point;
}
