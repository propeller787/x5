#pragma once
#include "interface/msg/car_information.hpp"
#include "interface/msg/hex_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <mutex>
#include <utils/singleton.h>
#include <vector>
#include <muduo/base/SignalSlot.h>
#include <interface/srv/id_controls.hpp>
#include <interface/srv/iccid_controls.hpp>
namespace sensor {
class Global {
public:
    using CarinfoCB = std::function<void(const interface::msg::CarInformation)>;

private:
    Global(void);

private:
    void loadParametersFromYAML(const std::string& file_path);
    void printParameters();
    void handleIdControls(const std::shared_ptr<interface::srv::IdControls::Request> request,
        std::shared_ptr<interface::srv::IdControls::Response> response);
    void handleIccidControls(const std::shared_ptr<interface::srv::IccidControls::Request> request,
        std::shared_ptr<interface::srv::IccidControls::Response> response);
public:
    static Global* getInstance()
    {
        // 双检锁
        if (instance == nullptr) {
            std::lock_guard<std::mutex> lock(mtx); // 加锁保证线程安全
            if (instance == nullptr) {
                instance = new Global();
            }
        }
        return instance;
    }

    void registerCarinfoCB(CarinfoCB cb);

    void onCarInforResult(const interface::msg::CarInformation& msg);

    void sendHexData(const interface::msg::HexData& msg);

    /** 从 SN 解析电池容量，格式: QX5AR10CA01260100001（第6-7位），失败返回 -1 */
    static int parseBatteryCapacityFromSn(const std::string& sn);
    int getBatteryCapacity() const { return battery_capacity_; }

    std::shared_ptr<rclcpp::Node> node_ { nullptr };
    interface::msg::CarInformation car_information_;
    
private:
    rclcpp::Service<interface::srv::IdControls>::SharedPtr id_controls_ { nullptr };
    rclcpp::Service<interface::srv::IccidControls>::SharedPtr iccid_controls_ { nullptr };
    int id_ = 0;
    std::string sn_file_path_ = "/opt/freego/vin";
    std::string sim_file_path_ = "/opt/freego/iccid";
    std::string sn_ = "Unknown";
    std::string iccid_ = "Unknown";
    int battery_capacity_ = -1;
    static Global* instance;
    static std::mutex mtx;

    std::shared_ptr<rclcpp::Subscription<interface::msg::CarInformation>> carInfoReceiverSpter_ { nullptr };

    std::vector<muduo::Slot> slots_carinfo_;
    muduo::Signal<void(const interface::msg::CarInformation&)> signal_carinfo_;

    rclcpp::Publisher<interface::msg::HexData>::SharedPtr hex_data_pub_;
};
}