#pragma once

#include "control/define.h"
#include "rclcpp/rclcpp.hpp"
#include "tft/tft_module.h"
#include "tft/tft_command_queue.h"
#include <atomic>
#include <chrono>
#include <fstream>
#include <interface/msg/named_status.hpp>
#include <mutex>
#include <thread>
#include <utils/singleton.h>
#include <vector>
namespace tft_service {
enum class LINGUISTIC_TYPE {
    CN, // 中文（中国）
    EN, // 英文（美国）
    DE, // 德文（德国）
    RU, // 俄文（俄国）
    JP, // 日语（日本）
    FR // 法语（法国）
};

enum class TFT_TYPE {
    STANDARD, // 标准
    HEALTH, // 健康检测中
    CRUISE, // 巡航
    AUTOMATIC_RETRACTION, // 回充
    OBSTACLE_AVOIDANCE, // 避障
    VIDEO, // 视频
    ALARM, // 预警
    SETTING // 设置
};

class Global {
private:
    Global(void);

public:
    ~Global();
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
    TFT_TYPE getType() { return type_; }
    void setType(TFT_TYPE type) { type_ = type; }
    LINGUISTIC_TYPE getLaType() { return la_type_; }
    void setLaType(LINGUISTIC_TYPE type) { la_type_ = type; }

    float getSpeedkmh() { return speedkmh_; }
    void setSpeedkmh(float speed) { speedkmh_ = speed; }

    bool getHealthStatus() { return health_status_; }
    void setHealthStatus(bool status);

    bool getCruiseStatus() { return cruise_status_; }
    void setCruiseStatus(bool status);

    bool getAutomaticRecharge() { return automatic_recharge_; }
    void setAutomaticRecharge(bool status);

    bool getSafeMode() { return safe_mode_; }
    void setSafeMode(bool status);

    bool getSetting() { return setting_; }
    void setSetting(bool status);

    void setElectricQuantity(int16_t electricquantity);
    // 网络检测相关方法
    bool getNetworkStatus() { return network_status_; }
    void setNetworkStatus(bool status);
    void startNetworkCheck();
    void stopNetworkCheck();
    bool checkNetworkConnectivity();

    /**
     * @brief 获取当前日期字符串
     * @param is_chinese 是否使用中文格式，true为中文，false为英文
     * @return 格式化的日期字符串
     */
    std::string getCurrentDateString();

    /**
     * @brief 获取当前时间字符串
     * @return 格式化的时间字符串 (HH : MM)
     */
    std::string getCurrentTimeString();

    /**
     * @brief 发送电量显示地址
     * @param index 电量显示地址  int16_t
     * @param x 电量显示地址  int16_t
     * @param y 电量显示地址  int16_t
     */
    void sendElectricIndex(int16_t index, int16_t x, int16_t y);
public:
    // 发送指定页面通过(ID)
    void sendPageId(int16_t id);
    // 发送指定地址数据
    void sendAddressData(uint16_t address, int16_t data);
    // 发送文本数据
    void sendTextData(uint16_t address, std::string text);

    // 发送电量
    void sendElectricQuantity();
    // 发送左上角日期
    void sendLeftDate();
    // 发送日期
    void sendDate();
    // 发送时间
    void sendTime();
    // 发送text
    void sendTextDemo();
    // 清除文本
    void clearText();

    // 发送网络
    void sendNetworkStatus();

    std::shared_ptr<TftModule> getTftModule();
    std::shared_ptr<TftCommandQueue> getCommandQueue();

public:
    // 发送页面抬头
    void sendPageHeader();

public:
    std::shared_ptr<rclcpp::Node> node_ { nullptr };

private:
    static Global* instance;
    static std::mutex mtx; // 互斥锁，保证线程安全

    TFT_TYPE type_ { TFT_TYPE::STANDARD };
    std::shared_ptr<TftModule> tft_module_;
    std::shared_ptr<TftCommandQueue> command_queue_;
    LINGUISTIC_TYPE la_type_ { LINGUISTIC_TYPE::EN };

    std::string date_buf_ { "" };
    std::string date_general_buf_ { "" };
    std::string time_buf_ { "" };
    int16_t electricquantity_ { 0 }; // 电量百分比
    float speedkmh_ { 0.0 }; // 速度km/h

    // 上个页面
    int16_t last_page_id_ { -1 }; // 上个页面ID
    // 当前页面
    int16_t current_page_id_ { -1 }; // 当前页面ID
    rclcpp::Publisher<interface::msg::NamedStatus>::SharedPtr named_status_pub_ { nullptr };
    bool health_status_ { false };
    bool cruise_status_ { false };
    bool automatic_recharge_ { false };
    bool safe_mode_ { false };
    bool setting_ { false };

    // 网络检测相关成员变量
    bool network_status_ { false };
    std::thread network_check_thread_;
    std::atomic<bool> network_check_running_ { false };
    std::atomic<bool> network_check_stop_requested_ { false };
    int NETWORK_CHECK_INTERVAL = 30; // 5秒检测间隔
};
}