#pragma once
#include "interface/msg/car_information.hpp"
#include "interface/msg/hex_data.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <mutex>
#include <utils/singleton.h>
#include <vector>
#include <muduo/base/SignalSlot.h>

namespace sensor {
class Global {
public:
    using CarinfoCB = std::function<void(const interface::msg::CarInformation)>;

private:
    Global(void);

private:
    void loadParametersFromYAML(const std::string& file_path);
    void printParameters();

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

    void onCarInforResult(const interface::msg::CarInformation msg);
    
    void sendHexData(const interface::msg::HexData msg);

public:
    std::shared_ptr<rclcpp::Node> node_ { nullptr };
    interface::msg::CarInformation car_information_;

private:
    static Global* instance;
    static std::mutex mtx;

    std::shared_ptr<rclcpp::Subscription<interface::msg::CarInformation>> carInfoReceiverSpter_ { nullptr };

    std::vector<muduo::Slot> slots_carinfo_;
    muduo::Signal<void(const interface::msg::CarInformation&)> signal_carinfo_;

    rclcpp::Publisher<interface::msg::HexData>::SharedPtr hex_data_pub_;
};
}