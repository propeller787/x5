/**
 * @file tft_user_config.h
 * @brief TFT 用户配置：用户名、车辆模式、声音、版本与升级状态，配置文件持久化，启动加载、修改实时写回。
 */

#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <interface/srv/id_controls.hpp>
#include <interface/srv/bluetooth_mac.hpp>
#include <rclcpp/node.hpp>

namespace tft_service {

/** 车辆模式：1 舒适 2 运动 */
enum class VehicleMode : int { Comfort = 1, Sport = 2 };

/** 语言：0 中文 1 英文 */
enum class Language : int { CN = 0, EN = 1 };

/**
 * @brief 用户配置单例：从 YAML 文件加载/保存，字段修改时实时写回文件。
 */
class TftUserConfig {
public:
    static TftUserConfig& instance();

    /** 配置文件路径（默认 /opt/freego/user_config.yaml） */
    std::string getConfigPath() const;

    /** 启动时调用：从文件加载，文件不存在则用默认值并写回 */
    bool load();
    /** 将当前状态写回配置文件（每次 set 后自动调用） */
    bool save();
    std::string getSnCode() const;
    std::string getBluetoothMac() const;
    void setSnCode(const std::string& sn);
    void setBluetoothMac(const std::string& bluetooth_mac);

    // --- 用户名 ---
    std::string getUsername() const;
    void setUsername(const std::string& v);

    // --- 车辆模式：1 舒适 2 运动 ---
    int getVehicleMode() const;
    void setVehicleMode(int v);

    // --- 声音开关：0 关闭 1 打开 ---
    int getSoundOn() const;
    void setSoundOn(int v);

    // --- 语言：0 中文 1 英文 ---
    Language getLanguage() const;
    void setLanguage(Language v);

    // --- 当前软件版本号 ---
    std::string getSoftwareVersion() const;
    void setSoftwareVersion(const std::string& v);

    // --- 软件包可升级状态：0 不可升级 1 可升级 ---
    int getUpgradeAvailable() const;
    void setUpgradeAvailable(int v);

    // --- 软件包可下载状态：0 不可下载 1 可下载 ---
    int getDownloadAvailable() const;
    void setDownloadAvailable(int v);

    // --- app下载界面状态：0 第一次开机 1 暂不下载 2 已下载下一步 ---
    int getAppDownloadPageState() const;
    void setAppDownloadPageState(int v);

    // --- app绑定界面：0 第一次开机 1 暂不绑定 2 已绑定(通过云端下发) ---
    int getAppBindPageState() const;
    void setAppBindPageState(int v);

    // --- 当前版本号 ---
    std::string getCurrentVersion() const;
    void setCurrentVersion(const std::string& v);

    // 记录用户隐私协议内容
    std::string getPrivacyPolicy();
    void setPrivacyPolicy(const std::string& v);

    // --- 当前版本号 ---
    bool getOtaNewPackageFlag() const;
    void setOtaNewPackageFlag(const bool& v);
    std::string getOtaUpgradeFailureDescriptionCn() const;
    std::string getOtaUpgradeFailureDescriptionEn() const;
    void setOtaUpgradeFailureDescriptionCn(const std::string& v);
    void setOtaUpgradeFailureDescriptionEn(const std::string& v);

    std::string getOtaUpgradeFailureDescription() const;

    // --- 是否启动followme：0 关闭 1 打开 ---
    int getFollowmeEnabled() const;
    void setFollowmeEnabled(int v);

    // --- 安全预警开关：0 关闭 1 打开 ---
    int getSafetyWarningEnabled() const;
    void setSafetyWarningEnabled(int v);

    // --- 档位记录：1-4 档 ---
    int getGearPosition() const;
    void setGearPosition(int v);

    // --- app下载码(sn+蓝牙mac，初始化时通过 idControlService / bluetoothMacService 获取) ---
    std::string getAppDownloadCode() const;

    std::string getSettingPageBindingQrCode() const;

    TftUserConfig(const TftUserConfig&) = delete;
    TftUserConfig& operator=(const TftUserConfig&) = delete;

private:
    TftUserConfig();
    mutable std::mutex mutex_;
    std::string config_path_;

    /** 未绑定时与 language_ 一致：中文「未绑定」/ 英文 Unbound，由构造函数与 normalize 维护 */
    std::string username_;
    int vehicle_mode_{1};
    int sound_on_{1};
    Language language_{Language::CN};
    std::string software_version_;
    int upgrade_available_{0};
    int download_available_{0};
    int app_download_page_state_{0};  // 0 第一次开机 1 暂不下载 2 已下载下一步
    int app_bind_page_state_{0};      // 0 第一次开机 1 暂不绑定 2 已绑定(通过云端下发)
    std::string app_bind_code_ {};       // app绑定码
    std::string app_download_code_ {"http://www.freegoai.com"}; // app下载码(sn+mac地址)
    std::string sn_code_ {}; // sn码
    std::string bluetooth_mac_ {}; // 蓝牙mac地址
    // 安全预警开关
    int safety_warning_enabled_{1};  // 0 关闭 1 打开
    // 记录档位
    int gear_position_{1};  // 1档 2档 3档 4档
    // 是否启动followme
    int followme_enabled_{0};

    std::string privacy_policy_{""};

    // ota相关 
    //当前版本号
    std::string current_version_ {};
    bool ota_new_package_flag_{false};
    void setConfigPathFromEnv();

    // 升级失败 中文描述（写入 user_config.yaml 记录）
    std::string ota_upgrade_failure_description_cn_ {"错误信息已上报，请稍后重试或联系客服400-4321-1234寻求帮助。"};
    // 升级失败 英文描述（写入 user_config.yaml 记录）
    std::string ota_upgrade_failure_description_en_ {"Error information has been reported, please try again later or contact customer service for assistance."};
};

}  // namespace tft_service
