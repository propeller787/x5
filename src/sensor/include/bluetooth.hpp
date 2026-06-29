#pragma once

#include <rclcpp/rclcpp.hpp>
#include <interface/msg/bluetooth_status.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>


namespace sensor::bluetooth {

class BluetoothComponent : public rclcpp::Node {
public:
    BluetoothComponent(const rclcpp::NodeOptions& options);
    ~BluetoothComponent();

private:
    struct BluetoothDevice {
        std::string name;     // 设备名称
        int rssi = -1;          // 信号强度
        std::string address;  // 设备地址
    };

    bool setupDevice();
    void onTimer();
    void workerThreadFunction();

    bool isConnected();
    bool toConnected(const BluetoothDevice& device);
    bool getBluetoothStatus(BluetoothDevice& devices);
    bool subscriberUUID();
    bool requestConnect();
    // 处理蓝牙返回的16进制数据并转换为topic信息
    bool processBluetoothHexData(const std::string& hex_data);
    
    // 配置文件相关方法
    bool loadMacAddressFromConfig();
    bool saveMacAddressToConfig(const std::string& mac_address);

    int device_fd_;
    bool isSubscribed_ = false;
    std::string bluetooth_device_path_;
    std::atomic<bool> worker_running_{false};
    std::thread worker_thread_;
    rclcpp::Publisher<interface::msg::BluetoothStatus>::SharedPtr bluetooth_status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 存储上一次的蓝牙状态
    std::shared_ptr<interface::msg::BluetoothStatus> last_status_;
    int request_connect_fail_count_ = 0;  // 记录 requestConnect 连续失败次数
    
    std::string config_file_path_ = "/opt/freego/config/bluetoothMac";       // 配置文件路径
    std::string saved_mac_address_;      // 保存的蓝牙MAC地址
};

} // namespace sensor::bluetooth
