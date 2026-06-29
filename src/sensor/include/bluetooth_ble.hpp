#pragma once

#include <rclcpp/rclcpp.hpp>
#include <interface/msg/bluetooth_status.hpp>
#include <memory>
#include <string>
#include <chrono>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <glib.h>
#include <gio/gio.h>

namespace sensor::bluetooth::_3568 {

class BluetoothBleComponent : public rclcpp::Node {
public:
    BluetoothBleComponent(const rclcpp::NodeOptions& options);
    ~BluetoothBleComponent();

private:
    struct BluetoothDevice {
        std::string name;
        std::string address;
        int rssi = 0;
        std::string path;
    };

    // D-Bus 相关
    std::string findAdapter();  // 查找适配器路径
    bool connectToDBus();   // 连接到D-Bus
    bool setDiscoveryFilter(const std::string& adapter_path);  // 设置搜索过滤器
    
    bool startDiscovery(const std::string& adapter_path);   // 开始搜索
    bool stopDiscovery(const std::string& adapter_path);    // 停止搜索
    bool connectDevice(const std::string& device_path);     // 连接设备
    bool disconnectDevice(const std::string& device_path);  // 断开设备连接 
    void findCharacteristics(const std::string& device_path);// 查找设备特征
    bool startNotify(const std::string& char_path);          // 开始通知
    bool writeCharacteristic(const std::string& char_path, const unsigned char* data, size_t len);// 写入特征值
    
    // 订阅信号
    void subscribeSignals();
    // 信号处理回调
    static void onPropertiesChanged(GDBusConnection* conn, const gchar* sender_name,
                                    const gchar* object_path, const gchar* interface_name,
                                    const gchar* signal_name, GVariant* parameters, gpointer user_data);
    static void onInterfacesAdded(GDBusConnection* conn, const gchar* sender_name,
                                  const gchar* object_path, const gchar* interface_name,
                                  const gchar* signal_name, GVariant* parameters, gpointer user_data);

    // 辅助方法
    GVariant* dbusCallSync(const char* dest, const char* path, const char* iface,
                           const char* method, GVariant* params, GError** error,
                           int timeout_ms = 5000);
    GVariant* getProperty(const char* path, const char* iface, const char* prop);
    bool setProperty(const char* path, const char* iface, const char* prop, GVariant* value);

    std::string findDeviceByMac(const std::string& mac_address);  // 按MAC地址查找设备路径
    static std::string toLower(const std::string& str);  // 字符串转小写
    // 线程函数
    void mainLoopThreadFunction();
    // 定时器回调函数（静态）
    static gboolean onTimeoutCallback(gpointer user_data);

    // 业务函数


    // 数据解析
    void parseAndPublishData(const unsigned char* data, size_t len);


    // D-Bus 连接和主循环
    GDBusConnection* dbus_conn_ = nullptr;
    GMainLoop* main_loop_ = nullptr;
    std::thread main_loop_thread_;
    guint timeout_id_ = 0;  // 定时器 ID

    // 状态标志
    std::atomic<bool> discovery_active_{false};
    std::atomic<bool> device_connected_{false};
    std::atomic<bool> should_exit_{false};
    std::atomic<bool> reconnecting_{false};  // 重连状态标志
    std::atomic<bool> need_find_characteristics_{false};  // 是否需要查找特征

    // 重连参数
    int reconnect_interval_ms_ = 3000;  // 重连间隔 3秒
    int max_reconnect_attempts_ = 3;    // 最大重连次数
    int max_disconnect_count_ = 10;     // 最大连续断开次数 10次
    std::atomic<int> reconnect_attempts_{0};  // 当前重连次数（原子变量，线程安全）
    std::atomic<int> reconnect_counter_{0};   // 重连检测计数器（原子变量，线程安全）
    std::atomic<int> disconnect_count_{0};    // 连续断开次数（原子变量，线程安全）

    // 目标设备信息 (需要线程保护)
    std::string target_path_;
    std::string tx_char_path_;
    std::string rx_char_path_;
    std::string adapter_path_;
    std::string target_mac_;  // 目标 MAC 地址（需要 mutex 保护）
    std::string pending_connect_path_;  // 待处理的连接路径（需要 mutex 保护）
    std::mutex path_mutex_;  // 保护路径变量和 target_mac_ 的互斥锁

    // 配置参数
    std::string target_name_key_ = "Freego";
    
    // 固定 UUID
    static constexpr const char* DEFAULT_TX_UUID = "0000fff2-0000-1000-8000-00805f9b34fb"; // APP发，模块收
    static constexpr const char* DEFAULT_RX_UUID = "0000fff1-0000-1000-8000-00805f9b34fb"; // 模块发，APP收
    
    unsigned char send_data_[8] = {0xAA, 0x55, 0xB1, 0xA1, 0xC0, 0x00, 0x02, 0x12};
    size_t send_len_ = 8;
    int send_interval_ms_ = 500;

    // 配置文件路径
    std::string config_file_path_ = "/opt/freego/config/bluetooth";

    // ROS2 相关
    rclcpp::Publisher<interface::msg::BluetoothStatus>::SharedPtr status_publisher_;
    std::shared_ptr<interface::msg::BluetoothStatus> last_status_;
    std::mutex status_mutex_;  // 保护 last_status_ 的互斥锁

    // 接收数据队列
    std::vector<std::vector<unsigned char>> receive_queue_;
    std::mutex queue_mutex_;
    static constexpr size_t MAX_QUEUE_SIZE = 1;  // 队列最大容量

    // 已发现设备缓存（避免重复处理）
    std::unordered_set<std::string> discovered_devices_;
    std::mutex device_cache_mutex_;  // 保护 discovered_devices_ 的互斥锁

    // 配置文件操作
    bool loadConfig();
    bool saveMacAddress(const std::string& mac_address);
};

} // namespace sensor::bluetooth::_3568