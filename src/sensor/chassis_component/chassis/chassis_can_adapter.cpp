#include "chassis_can_adapter.h"
#include "../../global.h"
#include "sensor_utils.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
using namespace sensor::chassis;
using namespace sensor::utils;

ChassisCanAdapter::ChassisCanAdapter(std::unordered_map<std::string, std::shared_ptr<CanTransreceiver>> devs)
    : CanAdapter(devs, "chassis")
{
    using namespace std::placeholders;
    message_unpack_table_ = {
        { CHASSIS_1_FRAME_ID, std::bind(&ChassisCanAdapter::onReceive214Info, this, _1, _2, _3) },
        { CHASSIS_2_FRAME_ID, std::bind(&ChassisCanAdapter::onReceive284Info, this, _1, _2, _3) },
    };
    std::map<uint16_t, unsigned int> canIdMap = {
        { CHASSIS_1_FRAME_ID, CAN_EFF_MASK },
        { CHASSIS_2_FRAME_ID, CAN_EFF_MASK },
    };
    // can_devs_["can2"]->filter(canIdMap);
    std::cout << " +++++++++++++++++++++++ " << std::endl;
}

void ChassisCanAdapter::onReceiveCanMessage(const std::string& dev, int fid, muduo::net::Buffer& buffer)
{
    // RCLCPP_INFO(logger_, "sh -- Rx:%s %04x %d %s", dev.c_str(), fid, buffer.readableBytes(),
    //     encodeToHexString(buffer.toStringPiece().as_string()).c_str());

    auto iter = message_unpack_table_.find(fid);
    if (iter == message_unpack_table_.end()) {
        // RCLCPP_WARN(logger_, "No Handle to unpack frame %04x", fid);
        return;
    }

    iter->second(dev, fid, buffer);
}
void ChassisCanAdapter::onReceiveGnssMessage(const interface::msg::Gnss::SharedPtr msg)
{
}

void ChassisCanAdapter::onRechargeStatusMessage(const interface::msg::RechargeStatus::SharedPtr msg)
{
}

void ChassisCanAdapter::onReceiveControlMessage(const geometry_msgs::msg::Twist::SharedPtr msg)
{
}

void ChassisCanAdapter::onRecieveSpeedbumpMessage(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
{
}

void ChassisCanAdapter::onRecievePaDaoMessage(const std::shared_ptr<std_msgs::msg::Int32> msg)
{
}

// 701
void ChassisCanAdapter::onReceive214Info(const std::string&, int fid, muduo::net::Buffer& buffer)
{
    // RCLCPP_INFO(logger_, "onReceive214Info --- %04x %d %s", fid, buffer.readableBytes(),
    //     encodeToHexString(buffer.toStringPiece().as_string()).c_str());
    const uint8_t* data = reinterpret_cast<const uint8_t*>(buffer.peek());

    auto message = geometry_msgs::msg::TwistStamped();
    message.header.stamp = this->now();
    message.header.frame_id = "base_link";

    interface::msg::Control controlMsg;
    auto now = muduo::Timestamp::now();
    uint8_t lowByte = data[2];
    uint8_t highByte = data[3];
    controlMsg.left_ratio = carCrtlGetMotorSpeed(highByte, lowByte);

    int16_t leftSpeed = highByte << 8 | lowByte;
    lowByte = data[4];
    highByte = data[5];

    controlMsg.hight_ratio = carCrtlGetMotorSpeed(highByte, lowByte);
    int16_t rightSpeed = highByte << 8 | lowByte;
    controlMsg.header.stamp.sec = now.secondsSinceEpoch();

    controlMsg.header.stamp.nanosec = (now.microSecondsSinceEpoch() % 1000000) * 1000;
    controlMsg.header.stamp = rclcpp::Time();
    controlMsg.header.frame_id = "base_link";
    controlMsg.electricquantity = electricquantity_;
    controlMsg.remainingmileage = remainingmileage_;
    distributionSpeed(leftSpeed, rightSpeed, controlMsg.vx, controlMsg.w);
    message.twist.linear.x = controlMsg.vx;
    message.twist.linear.y = 0.0;
    message.twist.linear.z = 0.0;
    message.twist.angular.x = 0.0;
    message.twist.angular.y = 0.0;
    message.twist.angular.z = controlMsg.w;
    control_car_spubs_->publish(message);
    control_car_pubs_->publish(controlMsg);
    const char* dataa = buffer.peek();
    size_t size = buffer.readableBytes();
    for (size_t i = 0; i < size; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<unsigned>(static_cast<unsigned char>(dataa[i])) << " ";
    }
    canSend("can0", (0x80021400 | CAN_EFF_FLAG), buffer);
}

// 假设我们有一个简化的电压-电量映射表（这些值是根据经验得到的，并不准确）
std::map<double, double> voltageToCapacityMap = {
    { 3.2 * 7, 0.0 }, // 当电压低于3.2V时，认为电量为0%
    { 3.4 * 7, 10.0 }, // 电压3.4V对应10%电量
    { 3.6 * 7, 30.0 }, // 电压3.6V对应30%电量
    { 3.7 * 7, 70.0 }, // 电压3.7V对应70%电量
    { 3.8 * 7, 90.0 }, // 额定电压3.8V对应90%电量
    { 4.2 * 7, 100.0 } // 当电压高于4.0V时（通常不会这样），认为电量为100%
};

// 线性插值函数，用于估算电压和映射表中两个点之间的电量
double linearInterpolate(double x, const std::pair<double, double>& p1, const std::pair<double, double>& p2)
{
    return p1.second + (x - p1.first) * (p2.second - p1.second) / (p2.first - p1.first);
}

// 根据电压估算电量的函数
double estimateCapacityFromVoltage(double voltage)
{
    // 找到电压所在的区间
    auto it = voltageToCapacityMap.lower_bound(voltage);

    if (it == voltageToCapacityMap.begin()) {
        // 电压低于映射表中的最小值
        return 0.0;
    } else if (it == voltageToCapacityMap.end()) {
        // 电压高于映射表中的最大值
        return 100.0;
    } else {
        // 电压位于两个映射点之间，进行线性插值
        auto prev = std::prev(it);
        return linearInterpolate(voltage, *prev, *it);
    }
}

void ChassisCanAdapter::onReceive284Info(const std::string&, int fid, muduo::net::Buffer& buffer)
{
    const char* data = buffer.peek();
    uint8_t byte2 = static_cast<uint8_t>(data[3]);
    uint8_t byte1 = static_cast<uint8_t>(data[2]);
    // 组合字节，形成709F
    int combined = ((byte2 << 8) | byte1);
    double estimeat = estimateCapacityFromVoltage(combined / 1000);
    int ele = mapBatteryPercent(estimeat);
    electricquantity_ = ele;
    remainingmileage_ = calcRemainingMileage(sensor::Global::getInstance()->getBatteryCapacity(), ele);
    canSend("can0", (0x80028400 | CAN_EFF_FLAG), buffer);
}

float ChassisCanAdapter::carCrtlGetMotorSpeed(uint8_t high, uint8_t low)
{
    auto speed = ((high << 8) | low);
    return speed / 17.0f / 8.2f;
}

void ChassisCanAdapter::sendCarInfo()
{
}

void ChassisCanAdapter::sendBattery()
{
}
