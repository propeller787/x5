#include "bluetooth_ble.hpp"
#include "utils/topic.h"
#include <cstring>
#include <fstream>
#include <sys/stat.h>

namespace sensor::bluetooth::_3568 {

// 辅助函数：字符串转小写
std::string BluetoothBleComponent::toLower(const std::string& str) {
    std::string result = str;
    for (auto& c : result) c = std::tolower(c);
    return result;
}

BluetoothBleComponent::BluetoothBleComponent(const rclcpp::NodeOptions& options)
    : Node("bluetooth_ble_3568", options) {
    RCLCPP_INFO(this->get_logger(), "Bluetooth BLE Component starting...");

    declare_parameter("send_interval_ms", 500);
    get_parameter("send_interval_ms", send_interval_ms_);
    target_name_key_ = "Freego";
    loadConfig();
    status_publisher_ = create_publisher<interface::msg::BluetoothStatus>(bluetoothStatusTopic, 10);

    if (!connectToDBus()) {
        RCLCPP_ERROR(get_logger(), "Failed to connect to D-Bus");
        return;
    }

    subscribeSignals();
    main_loop_ = g_main_loop_new(nullptr, FALSE);
    timeout_id_ = g_timeout_add(send_interval_ms_, onTimeoutCallback, this);
    main_loop_thread_ = std::thread(&BluetoothBleComponent::mainLoopThreadFunction, this);
}

BluetoothBleComponent::~BluetoothBleComponent() {
    should_exit_ = true;
    
    // 移除定时器
    if (timeout_id_ > 0) {
        g_source_remove(timeout_id_);
        timeout_id_ = 0;
    }
    
    if (main_loop_) g_main_loop_quit(main_loop_);
    if (main_loop_thread_.joinable()) main_loop_thread_.join();

    std::string target_path;
    {
        std::lock_guard<std::mutex> lock(path_mutex_);
        target_path = target_path_;
    }
    if (!target_path.empty()) disconnectDevice(target_path);
    if (main_loop_) g_main_loop_unref(main_loop_);
    if (dbus_conn_) g_object_unref(dbus_conn_);
}

void BluetoothBleComponent::mainLoopThreadFunction() {
    g_main_loop_run(main_loop_);
}

// 定时器回调函数，处理发送、接收、状态发布和重连检测
gboolean BluetoothBleComponent::onTimeoutCallback(gpointer user_data) {
    BluetoothBleComponent* self = static_cast<BluetoothBleComponent*>(user_data);
    try {
        if (self->should_exit_) {
            return FALSE;  // 停止定时器
        }
        std::string adapter_path;
        {
            std::lock_guard<std::mutex> lock(self->path_mutex_);
            adapter_path = self->adapter_path_;
        }
        if (adapter_path.empty()) {
            std::string new_adapter = self->findAdapter();  // 无锁调用
            if (!new_adapter.empty()) {
                std::lock_guard<std::mutex> lock(self->path_mutex_);
                self->adapter_path_ = new_adapter;
                adapter_path = new_adapter;
                RCLCPP_INFO(self->get_logger(), "Bluetooth adapter detected: %s", adapter_path.c_str());
            }
        }
        
        if (!adapter_path.empty()) {
            // 初始化流程（仅首次执行）
            static bool initialization_done = false;
            if (!initialization_done) {
                RCLCPP_INFO(self->get_logger(), "Initializing Bluetooth adapter...");
                // 开启蓝牙电源
                GVariant *powered = g_variant_new_boolean(TRUE);
                bool result = self->setProperty(adapter_path.c_str(), "org.bluez.Adapter1", "Powered", powered);
                if (!result) { // 重新执行定时器 ，等待蓝牙电源开启完成
                    return TRUE;
                }
                
                RCLCPP_INFO(self->get_logger(), "Bluetooth power turned on");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // 设置搜索过滤器
                if (!self->setDiscoveryFilter(adapter_path)) {
                    RCLCPP_WARN(self->get_logger(), "Failed to set discovery filter, using default settings");
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // 优先尝试使用配置的 MAC 地址进行连接
                std::string target_mac;
                {
                    std::lock_guard<std::mutex> lock(self->path_mutex_);
                    target_mac = self->target_mac_;
                }
                
                if (!target_mac.empty()) {
                    std::string device_path = self->findDeviceByMac(target_mac);
                    if (!device_path.empty() && self->connectDevice(device_path)) {
                        initialization_done = true;
                    } 
                }
                
                // 如果MAC连接失败或没有配置MAC，开始搜索设备
                if (!initialization_done) {
                    RCLCPP_INFO(self->get_logger(), "Starting BLE scan...");
                    self->startDiscovery(adapter_path);
                }
                initialization_done = true;
            }
        }
        
        // 0.5 处理延迟的特征查找（连接成功后）- 使用原子交换避免竞态
        bool need_find = self->need_find_characteristics_.exchange(false);
        if (need_find) {
            std::string target_path, rx_path, tx_path;
            {
                std::lock_guard<std::mutex> lock(self->path_mutex_);
                target_path = self->target_path_;
                rx_path = self->rx_char_path_;
                tx_path = self->tx_char_path_;
            }
            if (!target_path.empty()) {
                
                self->findCharacteristics(target_path);  // 无锁调用
                
                // 重新获取更新后的路径
                {
                    std::lock_guard<std::mutex> lock(self->path_mutex_);
                    rx_path = self->rx_char_path_;
                    tx_path = self->tx_char_path_;
                }
                
                if (!rx_path.empty()) {
                    if (self->startNotify(rx_path)) {
                        RCLCPP_INFO(self->get_logger(), "TX: %s, RX: %s, notifications enabled", 
                                   tx_path.c_str(), rx_path.c_str());
                    } else {
                        RCLCPP_WARN(self->get_logger(), "Failed to start RX notifications");
                        self->need_find_characteristics_ = true;
                    }
                } else {
                    // TX/RX未找到，设置重试标志，下次定时器周期重新搜索
                    RCLCPP_WARN(self->get_logger(), "TX/RX characteristics not found, will retry next cycle");
                    self->need_find_characteristics_ = true;
                }
            }
        }
        
        std::string pending_connect_path;
        {
            std::lock_guard<std::mutex> lock(self->path_mutex_);
            pending_connect_path = self->pending_connect_path_;
            if (!pending_connect_path.empty()) {
                self->pending_connect_path_.clear();  // 立即清除，防止重复处理
            }
        }
        if (!pending_connect_path.empty() && !self->device_connected_) {
            if (self->discovery_active_ && !adapter_path.empty()) {
                self->stopDiscovery(adapter_path);
            }
            self->connectDevice(pending_connect_path);
        }
        
        // 1. 先处理接收队列（优先处理上一次发送的回执）
        std::vector<unsigned char> data;
        {
            std::lock_guard<std::mutex> lock(self->queue_mutex_);
            if (!self->receive_queue_.empty()) {
                data = std::move(self->receive_queue_.front());
                self->receive_queue_.erase(self->receive_queue_.begin());
            }
        }
        if (!data.empty()) {
            self->parseAndPublishData(data.data(), data.size());
        }
        
        // 2. 发送数据
        if (self->device_connected_) {
            std::string tx_path;
            {
                std::lock_guard<std::mutex> lock(self->path_mutex_);
                tx_path = self->tx_char_path_;
            }
            if (!tx_path.empty()) {
                self->writeCharacteristic(tx_path, self->send_data_, self->send_len_);
                RCLCPP_INFO(self->get_logger(), "Sent data: AA55B1A1C0000212");
            }
            else {
                RCLCPP_WARN(self->get_logger(), "No TX characteristic path found");
            }
        }
        
        // 3. 发布状态
        auto status = std::make_shared<interface::msg::BluetoothStatus>();
        status->status = self->device_connected_;
        self->status_publisher_->publish(*status);
        RCLCPP_INFO(self->get_logger(), "Published status: %d", status->status);
        {
            std::lock_guard<std::mutex> lock(self->status_mutex_);
            self->last_status_ = status;
        }
        // 4. 重连检测（连续收到max_disconnect_count次断开则判定需要重连）
        bool need_reconnect = false;
        std::string target_mac;
        {
            std::lock_guard<std::mutex> lock(self->path_mutex_);
            need_reconnect = (!self->device_connected_);
        }
        if (need_reconnect) {
            self->disconnect_count_.fetch_add(1);
        }
        else{
            self->disconnect_count_.store(0);
            self->reconnecting_ = false;
        }
        RCLCPP_INFO(self->get_logger(), "需要重连: %d, 断开次数: %d", need_reconnect, self->disconnect_count_.load());
        // 执行重连逻辑 （连续5s断开进行重连 之后每50s重连）
        if (self->disconnect_count_.load() == self->max_disconnect_count_ || self->disconnect_count_.load() % self->max_disconnect_count_ * 10 == 0) {
            RCLCPP_INFO(self->get_logger(), "开始重连流程，停止搜索...");
            self->reconnecting_ = true;
            // 停止搜索
            if (!adapter_path.empty()) {
                self->stopDiscovery(adapter_path);
            }
            
            // 等待一段时间后重新开启搜索
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // 重新开启搜索
            if (!adapter_path.empty()) {
                self->startDiscovery(adapter_path);
                RCLCPP_INFO(self->get_logger(), "重连：搜索已重新开启");
            }
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(self->get_logger(), "Exception in onTimeoutCallback: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(self->get_logger(), "Unknown exception in onTimeoutCallback");
    }
    
    return TRUE; 
}

void BluetoothBleComponent::parseAndPublishData(const unsigned char* data, size_t len) {
    std::string hex_data;
    for (size_t i = 0; i < len; ++i) {
        char buf[4];
        snprintf(buf, sizeof(buf), "%02X", data[i]);
        hex_data += buf;
    }
    RCLCPP_INFO(get_logger(), "Received: %s", hex_data.c_str());

    try {
        if (hex_data.substr(0, 4) != "AA55" || hex_data.size() < 12) {
            RCLCPP_ERROR(get_logger(), "Invalid frame: %s", hex_data.c_str());
            return;
        }

        int data_len = std::stoi(hex_data.substr(10, 2), nullptr, 16);
        
        // 范围检查：防止 data_len 过大或为负数
        if (data_len < 0 || data_len > 100) {
            RCLCPP_ERROR(get_logger(), "Invalid data_len: %d (must be 0-100)", data_len);
            return;
        }
        
        if (12 + data_len * 2 + 4 != int(hex_data.size())) {
            RCLCPP_ERROR(get_logger(), "Length mismatch: expect=%d, actual=%zu", 12 + data_len * 2 + 4, hex_data.size());
            return;
        }

        int16_t voltage = static_cast<int16_t>(std::stoul(hex_data.substr(12, 2), nullptr, 16));
        bool key_status = std::stoul(hex_data.substr(14, 2), nullptr, 16) & 0x01;
        bool charging_status = std::stoul(hex_data.substr(16, 2), nullptr, 16) & 0x01;

        RCLCPP_INFO(get_logger(), "Voltage: %d, Key: %s, Charging: %s",
                    voltage, key_status ? "pressed" : "released", charging_status ? "yes" : "no");

        auto msg = std::make_shared<interface::msg::BluetoothStatus>();
        msg->status = device_connected_;
        msg->voltage = voltage;
        msg->key_status = key_status;
        msg->charging_status = charging_status;
        status_publisher_->publish(*msg);

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            last_status_ = msg;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Parse error: %s", e.what());
    }
}

bool BluetoothBleComponent::connectToDBus() {
    GError* error = nullptr;
    dbus_conn_ = g_bus_get_sync(G_BUS_TYPE_SYSTEM, nullptr, &error);
    if (!dbus_conn_) {
        RCLCPP_ERROR(get_logger(), "D-Bus connect failed: %s", error->message);
        g_error_free(error);
        return false;
    }
    return true;
}

std::string BluetoothBleComponent::findAdapter() {
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr, &err);
    if (!result) {
        if (err) g_error_free(err);
        return "";
    }

    GVariantIter* obj_iter = nullptr;
    g_variant_get(result, "(a{oa{sa{sv}}})", &obj_iter);
    if (!obj_iter) {
        g_variant_unref(result);
        return "";
    }

    const gchar* path = nullptr;
    GVariantIter* iface_iter = nullptr;
    while (g_variant_iter_loop(obj_iter, "{&oa{sa{sv}}}", &path, &iface_iter)) {
        if (!path) continue;
        const gchar* iface_name = nullptr;
        GVariantIter* prop_iter = nullptr;
        while (g_variant_iter_loop(iface_iter, "{&sa{sv}}", &iface_name, &prop_iter)) {
            if (iface_name && strcmp(iface_name, "org.bluez.Adapter1") == 0) {
                std::string adapter(path);
                g_variant_iter_free(obj_iter);
                g_variant_unref(result);
                return adapter;
            }
        }
    }
    g_variant_iter_free(obj_iter);
    g_variant_unref(result);
    return "";
}

// 设置搜索过滤器：只搜索 BLE 设备，主动搜索模式
bool BluetoothBleComponent::setDiscoveryFilter(const std::string& adapter_path) {
    GVariantBuilder builder;
    g_variant_builder_init(&builder, G_VARIANT_TYPE("a{sv}"));
    // 1. Transport: 只搜索 BLE 设备（低功耗蓝牙）
    g_variant_builder_add(&builder, "{sv}", "Transport", g_variant_new_string("le"));
    // 2. DuplicateData: TRUE - 允许重复数据，加快发现速度（主动扫描模式）
    g_variant_builder_add(&builder, "{sv}", "DuplicateData", g_variant_new_boolean(TRUE));
    // 3. RSSI threshold: 只显示信号强度 >= -120dBm 的设备
    g_variant_builder_add(&builder, "{sv}", "RSSI", g_variant_new_int16(-120));
    // 4. NamePrefix: 只显示名称包含 target_name_key_ 的设备
    // g_variant_builder_add(&builder, "{sv}", "Pattern", g_variant_new_string((target_name_key_ + "*").c_str()));
    
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", adapter_path.c_str(), 
                                        "org.bluez.Adapter1", "SetDiscoveryFilter",
                                        g_variant_new("(a{sv})", &builder), &err);
    
    if (err) {
        RCLCPP_ERROR(get_logger(), "SetDiscoveryFilter failed: %s", err->message);
        g_error_free(err);
        return false;
    }
    
    if (result) g_variant_unref(result);
    
    RCLCPP_INFO(get_logger(), "Discovery filter set: BLE only, active scan, RSSI >= -120dBm");
    return true;
}

bool BluetoothBleComponent::startDiscovery(const std::string& adapter_path) {
    if (discovery_active_) {
        RCLCPP_DEBUG(get_logger(), "Discovery already active, skipping StartDiscovery call");
        return true;
    }
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", adapter_path.c_str(), "org.bluez.Adapter1", "StartDiscovery", nullptr, &err);
    if (err) {
        RCLCPP_ERROR(get_logger(), "StartDiscovery failed: %s", err->message);
        g_error_free(err);
        return false;
    }
    if (result) g_variant_unref(result);
    discovery_active_ = true;
    return true;
}

bool BluetoothBleComponent::stopDiscovery(const std::string& adapter_path) {
    if (!discovery_active_) {
        return true;
    }
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", adapter_path.c_str(), "org.bluez.Adapter1", "StopDiscovery", nullptr, &err);
    if (result) g_variant_unref(result);
    if (err) {
        RCLCPP_ERROR(get_logger(), "StopDiscovery failed: %s", err->message);
        g_error_free(err);
        return false;
    }
    discovery_active_ = false;
    return true;
}

bool BluetoothBleComponent::connectDevice(const std::string& device_path) {
    RCLCPP_INFO(get_logger(), "Connecting: %s", device_path.c_str());
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", device_path.c_str(), "org.bluez.Device1", "Connect", nullptr, &err, 8000);
    RCLCPP_INFO(get_logger(), "Connect call returned");
    
    if (err) {
        RCLCPP_WARN(get_logger(), "Connect failed: %s", err->message);
        g_error_free(err);
        // 连接失败时，尝试取消可能存在的配对状态，清理设备缓存
        GError* cancel_err = nullptr;
        dbusCallSync("org.bluez", device_path.c_str(), "org.bluez.Device1", "CancelPairing", nullptr, &cancel_err);
        if (cancel_err) {
            RCLCPP_WARN(get_logger(), "CancelPairing failed: %s", cancel_err->message);
            g_error_free(cancel_err);
        }
        
        return false;
    }
    
    if (result) g_variant_unref(result);

    device_connected_ = true;
    target_path_ = device_path;

    GVariant* addr_var = getProperty(device_path.c_str(), "org.bluez.Device1", "Address");
    if (addr_var) {
        std::string mac_address = g_variant_get_string(addr_var, nullptr);
        g_variant_unref(addr_var);
        if (!mac_address.empty()) {
            saveMacAddress(mac_address);
            RCLCPP_INFO(get_logger(), "MAC saved, about to lock mutex");
            {
                std::lock_guard<std::mutex> lock(path_mutex_);
                target_mac_ = mac_address;
            }
            RCLCPP_INFO(get_logger(), "Mutex unlocked, about to return");
        }
    }
    RCLCPP_INFO(get_logger(), "connectDevice returning true");
    return true;
}

std::string BluetoothBleComponent::findDeviceByMac(const std::string& mac_address) {
    if (mac_address.empty()) return "";
    
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr, &err);
    if (err) {
        RCLCPP_ERROR(get_logger(), "GetManagedObjects failed: %s", err->message);
        g_error_free(err);
        return "";
    }
    
    if (!result) return "";
    
    GVariantIter* obj_iter = nullptr;
    g_variant_get(result, "(a{oa{sa{sv}}})", &obj_iter);
    if (!obj_iter) {
        g_variant_unref(result);
        return "";
    }
    
    const gchar* object_path = nullptr;
    GVariantIter* iface_iter = nullptr;
    while (g_variant_iter_loop(obj_iter, "{&oa{sa{sv}}}", &object_path, &iface_iter)) {
        if (!object_path) continue;
        const gchar* iface_name = nullptr;
        GVariantIter* prop_iter = nullptr;
        while (g_variant_iter_loop(iface_iter, "{&sa{sv}}", &iface_name, &prop_iter)) {
            if (iface_name && strcmp(iface_name, "org.bluez.Device1") == 0) {
                const gchar* prop_key = nullptr;
                GVariant* prop_value = nullptr;
                while (g_variant_iter_loop(prop_iter, "{&sv}", &prop_key, &prop_value)) {
                    if (prop_key && strcmp(prop_key, "Address") == 0) {
                        gchar* addr = g_variant_dup_string(prop_value, nullptr);
                        if (addr) {
                            std::string device_addr(addr);
                            g_free(addr);
                            
                            // 比较MAC地址（不区分大小写）
                            std::string mac_upper = mac_address;
                            std::string device_addr_upper = device_addr;
                            for (auto& c : mac_upper) c = std::toupper(c);
                            for (auto& c : device_addr_upper) c = std::toupper(c);
                            
                            if (mac_upper == device_addr_upper) {
                                std::string result_path(object_path);
                                g_variant_iter_free(obj_iter);
                                g_variant_unref(result);
                                return result_path;
                            }
                        }
                    }
                    // g_variant_iter_loop 自动释放 prop_value，无需手动释放
                }
            }
        }
    }
    g_variant_iter_free(obj_iter);
    g_variant_unref(result);
    return "";
}

bool BluetoothBleComponent::disconnectDevice(const std::string& device_path) {
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", device_path.c_str(), "org.bluez.Device1", "Disconnect", nullptr, &err);
    if (result) g_variant_unref(result);
    if (err) g_error_free(err);
    device_connected_ = false;
    return true;
}

void BluetoothBleComponent::findCharacteristics(const std::string& device_path) {
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects", nullptr, &err);
    if (!result) {
        if (err) g_error_free(err);
        return;
    }

    std::string tx_path, rx_path;
    size_t device_path_len = device_path.length();
    GVariantIter* obj_iter = nullptr;
    g_variant_get(result, "(a{oa{sa{sv}}})", &obj_iter);
    if (!obj_iter) {
        g_variant_unref(result);
        return;
    }
    const gchar* path = nullptr;
    GVariantIter* iface_iter = nullptr;
    
    // 遍历所有对象，找到目标设备下的GATT特征
    while (g_variant_iter_loop(obj_iter, "{&oa{sa{sv}}}", &path, &iface_iter)) {
        if (!path) continue;
        if (strncmp(path, device_path.c_str(), device_path_len) != 0) {
            continue;
        }
        const gchar* iface_name = nullptr;
        GVariantIter* prop_iter = nullptr;
        while (g_variant_iter_loop(iface_iter, "{&sa{sv}}", &iface_name, &prop_iter)) {
            if (!iface_name || strcmp(iface_name, "org.bluez.GattCharacteristic1") != 0) {
                continue;
            }

            // 直接从属性迭代器中提取UUID，避免额外的D-Bus调用
            const gchar* prop_key = nullptr;
            GVariant* prop_value = nullptr;
            while (g_variant_iter_loop(prop_iter, "{&sv}", &prop_key, &prop_value)) {
                if (prop_key && strcmp(prop_key, "UUID") == 0) {
                    gchar* uuid_str = g_variant_dup_string(prop_value, nullptr);
                    if (uuid_str) {
                        if (strcasecmp(uuid_str, DEFAULT_TX_UUID) == 0) {
                            tx_path = path;
                        } else if (strcasecmp(uuid_str, DEFAULT_RX_UUID) == 0) {
                            rx_path = path;
                        }
                        g_free(uuid_str);
                        
                        // 优化：找到TX和RX后立即退出，无需继续遍历
                        if (!tx_path.empty() && !rx_path.empty()) {
                            g_variant_iter_free(obj_iter);
                            g_variant_unref(result);
                            
                            // 一次性更新共享状态
                            std::lock_guard<std::mutex> lock(path_mutex_);
                            tx_char_path_ = tx_path;
                            rx_char_path_ = rx_path;
                            return;
                        }
                    }
                }
                // g_variant_iter_loop 自动释放 prop_value
            }
        }
    }
    
    g_variant_iter_free(obj_iter);
    g_variant_unref(result);

    // 更新找到的特征路径
    if (!tx_path.empty() || !rx_path.empty()) {
        std::lock_guard<std::mutex> lock(path_mutex_);
        if (!tx_path.empty()) tx_char_path_ = tx_path;
        if (!rx_path.empty()) rx_char_path_ = rx_path;
    }
}

bool BluetoothBleComponent::startNotify(const std::string& char_path) {
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", char_path.c_str(), "org.bluez.GattCharacteristic1", "StartNotify", nullptr, &err);
    if (err) {
        RCLCPP_ERROR(get_logger(), "StartNotify failed: %s", err->message);
        g_error_free(err);
        return false;
    }
    if (result) g_variant_unref(result);
    return true;
}

bool BluetoothBleComponent::writeCharacteristic(const std::string& char_path, const unsigned char* data, size_t len) {
    GVariantBuilder builder;
    g_variant_builder_init(&builder, G_VARIANT_TYPE("ay"));
    for (size_t i = 0; i < len; ++i) g_variant_builder_add(&builder, "y", data[i]);

    GVariantBuilder opts;
    g_variant_builder_init(&opts, G_VARIANT_TYPE("a{sv}"));

    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", char_path.c_str(), "org.bluez.GattCharacteristic1", "WriteValue",
                                    g_variant_new("(aya{sv})", &builder, &opts), &err);
    if (err) g_error_free(err);
    if (result) g_variant_unref(result);
    return !err;
}

void BluetoothBleComponent::subscribeSignals() {
    g_dbus_connection_signal_subscribe(dbus_conn_, "org.bluez", "org.freedesktop.DBus.Properties",
                                       "PropertiesChanged", nullptr, nullptr, G_DBUS_SIGNAL_FLAGS_NONE,
                                       onPropertiesChanged, this, nullptr);
    g_dbus_connection_signal_subscribe(dbus_conn_, "org.bluez", "org.freedesktop.DBus.ObjectManager",
                                       "InterfacesAdded", nullptr, nullptr, G_DBUS_SIGNAL_FLAGS_NONE,
                                       onInterfacesAdded, this, nullptr);
}

void BluetoothBleComponent::onPropertiesChanged(GDBusConnection* conn, const gchar* sender_name,
                                                const gchar* object_path, const gchar* interface_name,
                                                const gchar* signal_name, GVariant* parameters, gpointer user_data) {
    (void)conn; (void)sender_name; (void)interface_name; (void)signal_name;

    BluetoothBleComponent* self = static_cast<BluetoothBleComponent*>(user_data);

    GVariantIter* changed_iter = nullptr;
    
    try {
        const char* iface;
        g_variant_get(parameters, "(&sa{sv}as)", &iface, &changed_iter, nullptr);

        if (strcmp(iface, "org.bluez.Device1") == 0) {
            const gchar* key = nullptr;
            GVariant* value = nullptr;
            std::string name;
            bool connected_changed = false;
            gboolean is_connected = FALSE;

            while (g_variant_iter_loop(changed_iter, "{&sv}", &key, &value)) {
                if (key && strcmp(key, "Name") == 0) {
                    name = g_variant_get_string(value, nullptr);
                } else if (key && strcmp(key, "Connected") == 0) {
                    is_connected = g_variant_get_boolean(value);
                    connected_changed = true;
                }
            }

            if (connected_changed) {
                self->device_connected_ = is_connected;
                if (is_connected) {
                    RCLCPP_INFO(self->get_logger(), "Connected: %s", object_path);
                    {
                        std::lock_guard<std::mutex> lock(self->path_mutex_);
                        self->target_path_ = object_path;
                    }
                    self->need_find_characteristics_ = true;
                } else {
                    RCLCPP_WARN(self->get_logger(), "Disconnected: %s", object_path);
                    std::lock_guard<std::mutex> lock(self->path_mutex_);
                    self->tx_char_path_.clear();
                    self->rx_char_path_.clear();
                }
            }

            if (!name.empty() && self->discovery_active_) {
                if (toLower(name).find(toLower(self->target_name_key_)) != std::string::npos) {
                    RCLCPP_INFO(self->get_logger(), "Found target: %s (%s)", object_path, name.c_str());
                    {
                        std::lock_guard<std::mutex> lock(self->path_mutex_);
                        self->target_path_ = object_path;
                        self->pending_connect_path_ = object_path;
                    }
                    
                    // 立即停止搜索，减少资源消耗
                    std::string adapter;
                    {
                        std::lock_guard<std::mutex> lock(self->path_mutex_);
                        adapter = self->adapter_path_;
                    }
                    if (!adapter.empty()) {
                        self->stopDiscovery(adapter);
                        RCLCPP_INFO(self->get_logger(), "Discovery stopped after finding target device");
                    }
                }
            }
        } else if (strcmp(iface, "org.bluez.GattCharacteristic1") == 0) {
            std::string rx_path;
            {
                std::lock_guard<std::mutex> lock(self->path_mutex_);
                rx_path = self->rx_char_path_;
            }
            if (rx_path == object_path) {
                const gchar* key = nullptr;
                GVariant* value = nullptr;
                while (g_variant_iter_loop(changed_iter, "{&sv}", &key, &value)) {
                    if (key && strcmp(key, "Value") == 0) {
                        gsize len;
                        const unsigned char* data = (const unsigned char*)g_variant_get_fixed_array(value, &len, sizeof(unsigned char));
                        if (data && len > 0) {
                            std::vector<unsigned char> buffer(data, data + len);
                            std::lock_guard<std::mutex> lock(self->queue_mutex_);
                            if (self->receive_queue_.size() < self->MAX_QUEUE_SIZE) {
                                self->receive_queue_.push_back(buffer);
                            }
                        }
                    }
                    // g_variant_iter_loop 自动释放 value，无需手动释放
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(self->get_logger(), "Exception in onPropertiesChanged: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(self->get_logger(), "Unknown exception in onPropertiesChanged");
    }
    
    // 确保资源清理
    if (changed_iter) {
        g_variant_iter_free(changed_iter);
    }
}

void BluetoothBleComponent::onInterfacesAdded(GDBusConnection* conn, const gchar* sender_name,
                                              const gchar* object_path, const gchar* interface_name,
                                              const gchar* signal_name, GVariant* parameters, gpointer user_data) {
    (void)conn; (void)sender_name; (void)object_path; (void)interface_name; (void)signal_name;

    BluetoothBleComponent* self = static_cast<BluetoothBleComponent*>(user_data);

    GVariantIter* iface_iter = nullptr;
    const gchar* path = nullptr;
    
    try {
        g_variant_get(parameters, "(&oa{sa{sv}})", &path, &iface_iter);
        if (!iface_iter) return;

        const gchar* iface_name = nullptr;
        GVariantIter* prop_iter = nullptr;
        bool found_device = false;
        while (g_variant_iter_loop(iface_iter, "{&sa{sv}}", &iface_name, &prop_iter)) {
            if (iface_name && strcmp(iface_name, "org.bluez.Device1") == 0) {
                found_device = true;
                const gchar* key = nullptr;
                GVariant* value = nullptr;
                std::string name, addr;
                int rssi = 0;

                while (g_variant_iter_loop(prop_iter, "{&sv}", &key, &value)) {
                    if (key && strcmp(key, "Name") == 0) name = g_variant_get_string(value, nullptr);
                    else if (key && strcmp(key, "Address") == 0) addr = g_variant_get_string(value, nullptr);
                    else if (key && strcmp(key, "RSSI") == 0) rssi = g_variant_get_int16(value);
                }

                if (name.empty()) name = "Unknown";
                RCLCPP_INFO(self->get_logger(), "Discovered device: %s | %s | RSSI: %d", 
                           addr.c_str(), name.c_str(), rssi);
                
                if (self->discovery_active_ && !name.empty() && name != "Unknown") {
                    if (toLower(name).find(toLower(self->target_name_key_)) != std::string::npos) {
                        {
                            std::lock_guard<std::mutex> lock(self->path_mutex_);
                            self->target_path_ = path;
                            self->pending_connect_path_ = path;
                            RCLCPP_INFO(self->get_logger(), "Found target device: %s (%s)", path, name.c_str());
                        }
                        // 立即停止搜索，减少资源消耗
                        if (self->discovery_active_) {
                            std::string adapter;
                            {
                                std::lock_guard<std::mutex> lock(self->path_mutex_);
                                adapter = self->adapter_path_;
                            }
                            if (!adapter.empty()) {
                                self->stopDiscovery(adapter);
                                RCLCPP_INFO(self->get_logger(), "Discovery stopped after finding target device");
                            }
                        }
                    }
                }
            }
        }
        if (!found_device) {
            RCLCPP_DEBUG(self->get_logger(), "InterfacesAdded received but no Device1 interface found");
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(self->get_logger(), "Exception in onInterfacesAdded: %s", e.what());
    } catch (...) {
        RCLCPP_ERROR(self->get_logger(), "Unknown exception in onInterfacesAdded");
    }
    
    // 确保资源清理
    if (iface_iter) {
        g_variant_iter_free(iface_iter);
    }
}

GVariant* BluetoothBleComponent::dbusCallSync(const char* dest, const char* path, const char* iface,
                                              const char* method, GVariant* params, GError** error,
                                              int timeout_ms) {
    return g_dbus_connection_call_sync(dbus_conn_, dest, path, iface, method, params, nullptr,
                                       G_DBUS_CALL_FLAGS_NONE, timeout_ms, nullptr, error);
}

GVariant* BluetoothBleComponent::getProperty(const char* path, const char* iface, const char* prop) {
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", path, "org.freedesktop.DBus.Properties", "Get",
                                    g_variant_new("(ss)", iface, prop), &err);
    if (err) {
        g_error_free(err);
        return nullptr;
    }
    GVariant* value;
    g_variant_get(result, "(v)", &value);
    g_variant_unref(result);
    return value;
}

bool BluetoothBleComponent::setProperty(const char* path, const char* iface, const char* prop, GVariant* value) {
    GError* err = nullptr;
    GVariant* result = dbusCallSync("org.bluez", path, "org.freedesktop.DBus.Properties", "Set",
                                    g_variant_new("(ssv)", iface, prop, value), &err);
    // 调用者负责释放传入的 value
    if (err) {
        RCLCPP_ERROR(get_logger(), "SetProperty failed: %s", err->message);
        g_error_free(err);
        return false;
    }
    if (result) g_variant_unref(result);
    return true;
}

bool BluetoothBleComponent::loadConfig() {
    std::ifstream config_file(config_file_path_);
    if (!config_file.is_open()) {
        RCLCPP_WARN(get_logger(), "Config not found: %s", config_file_path_.c_str());
        return false;
    }
    std::string line;
    if (std::getline(config_file, line)) {
        // 去除首尾空白字符
        line.erase(0, line.find_first_not_of(" \t\r\n"));
        line.erase(line.find_last_not_of(" \t\r\n") + 1);
        if (!line.empty()) {
            {
                std::lock_guard<std::mutex> lock(path_mutex_);
                target_mac_ = line;
            }
            RCLCPP_INFO(get_logger(), "Loaded MAC address from config: %s", line.c_str());
            return true;
        }
    }
    return false;
}

bool BluetoothBleComponent::saveMacAddress(const std::string& mac_address) {
    std::string config_dir = config_file_path_.substr(0, config_file_path_.find_last_of('/'));
    mkdir(config_dir.c_str(), 0755);

    std::ofstream config_file(config_file_path_);
    if (!config_file.is_open()) {
        RCLCPP_ERROR(get_logger(), "Failed to write config: %s", config_file_path_.c_str());
        return false;
    }
    config_file << mac_address;
    RCLCPP_INFO(get_logger(), "Saved MAC: %s", mac_address.c_str());
    return true;
}

} // namespace sensor::bluetooth::_3568