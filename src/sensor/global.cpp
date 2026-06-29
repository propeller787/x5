#include "global.h"
#include <rclcpp/rclcpp.hpp>
#include <utils/topic.h>
#include <net/if.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <random>
using std::placeholders::_1;
using std::placeholders::_2;
namespace sensor {
    static int read_sn(char* sn)
    {
        if (!sn) {
            return -1;
        }
    
        std::ifstream file("/opt/freego/config/sn");
        if (!file.is_open()) {
            return -1;
        }
    
        std::string line;
        if (std::getline(file, line)) {
            size_t copy_len = std::min(line.length(), static_cast<size_t>(255));
            strncpy(sn, line.c_str(), copy_len);
            sn[copy_len] = '\0';
            return copy_len;
        }
    
        return -1;
    }
    static int read_sim(char* iccid){
        if (!iccid) {
            return -1;
        }
    
        std::ifstream file("/opt/freego/config/iccid");
        if (!file.is_open()) {
            return -1;
        }
    
        std::string line;
        if (std::getline(file, line)) {
            size_t copy_len = std::min(line.length(), static_cast<size_t>(255));
            strncpy(iccid, line.c_str(), copy_len);
            iccid[copy_len] = '\0';
            return copy_len;
        }
        return -1;
    }

Global* Global::instance = nullptr;
std::mutex Global::mtx;
Global::Global(void)
{
    node_ = std::make_shared<rclcpp::Node>("sensor");
    hex_data_pub_ = node_->create_publisher<interface::msg::HexData>(errorEventTopic, 10);
    char snbuf[256];
    int sn_len = read_sn(snbuf);  // 通过data()获取底层数组指针
    if (sn_len > 0) {
        sn_ = std::string(snbuf, sn_len);
        battery_capacity_ = parseBatteryCapacityFromSn(sn_);
        if (battery_capacity_ >= 0) {
            RCLCPP_INFO(node_->get_logger(), "SN battery capacity: %d (from %s)", battery_capacity_, sn_.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to parse battery capacity from SN: %s", sn_.c_str());
        }
    }
    
    char iccidbuf[256];
    int iccid_len = read_sim(iccidbuf);
    if (iccid_len > 0) {
        iccid_ = std::string(iccidbuf, iccid_len);
    }
    
    iccid_controls_ = node_->create_service<interface::srv::IccidControls>(iccidControlService, std::bind(&Global::handleIccidControls, this, _1, _2));
    id_controls_ = node_->create_service<interface::srv::IdControls>(idControlService, std::bind(&Global::handleIdControls, this, _1, _2));
}

void Global::sendHexData(const interface::msg::HexData& msg)
{
    hex_data_pub_->publish(msg);
}

void Global::loadParametersFromYAML(const std::string& file_path)
{

}

void Global::printParameters()
{
    RCLCPP_INFO(node_->get_logger(), "Loaded parameters:");
    auto parameters_and_prefixes = node_->list_parameters({}, 1);
    for (const auto& parameter : parameters_and_prefixes.names) {
        RCLCPP_INFO(node_->get_logger(), "%s : %d", parameter.c_str(), node_->get_parameter(parameter).get_type());
    }
}

void Global::registerCarinfoCB(CarinfoCB cb)
{
    slots_carinfo_.emplace_back(signal_carinfo_.connect(std::move(cb)));
}

void Global::onCarInforResult(const interface::msg::CarInformation& msg)
{
    car_information_ = msg;
    signal_carinfo_.call(msg);
}

void Global::handleIdControls(const std::shared_ptr<interface::srv::IdControls::Request> request,
    std::shared_ptr<interface::srv::IdControls::Response> response)
{
    response->set__code(0);
    response->set__sn(sn_);
}

void Global::handleIccidControls(const std::shared_ptr<interface::srv::IccidControls::Request> request,
    std::shared_ptr<interface::srv::IccidControls::Response> response)
{
    response->set__code(0);
    response->set__iccid(iccid_);
}

int Global::parseBatteryCapacityFromSn(const std::string& sn)
{
    if (sn.empty() || sn == "Unknown") {
        return -1;
    }

    // SN 无分隔符，示例: QX5AR10CA01260100001
    // 布局: Q(1)+型号(3)+左右手(1)+电池容量(2)+区域(1)+工厂(1)+预留(2)+年月(4)+流水(5)
    constexpr size_t kBatteryCapacityOffset = 5;
    constexpr size_t kBatteryCapacityLength = 2;
    constexpr size_t kMinSnLength = kBatteryCapacityOffset + kBatteryCapacityLength;
    if (sn.length() < kMinSnLength) {
        return -1;
    }

    try {
        const int capacity = std::stoi(sn.substr(kBatteryCapacityOffset, kBatteryCapacityLength));
        return capacity > 0 ? capacity : -1;
    } catch (const std::exception&) {
        return -1;
    }
}

}