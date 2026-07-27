/**
 * @file tft_user_config.cpp
 * @brief TftUserConfig 实现：YAML 配置文件加载与实时保存。
 */

#include "tft_user_config.h"
#include <yaml-cpp/yaml.h>
#include <cstdlib>
#include <fstream>
#include <sys/stat.h>
#include <rclcpp/rclcpp.hpp>
#include <utils/topic.h>
#include <chrono>
#include <future>

namespace tft_service {

namespace {

const char* kUsername = "username";
const char* kVehicleMode = "vehicle_mode";
const char* kSoundOn = "sound_on";
const char* kSoftwareVersion = "software_version";
const char* kUpgradeAvailable = "upgrade_available";
const char* kDownloadAvailable = "download_available";
const char* kAppDownloadPageState = "app_download_page_state";
const char* kAppBindPageState = "app_bind_page_state";
const char* kSnCode = "sn_code";
const char* kBluetoothMac = "bluetooth_mac";
const char* kCurrentVersion = "current_version";
const char* kLanguage = "language";
const char* kFollowmeEnabled = "followme_enabled";
const char* kSafetyWarningEnabled = "safety_warning_enabled";
const char* kGearPosition = "gear_position";
const char* kPrivacyPolicy = "privacy_policy";

const char* kOtaUpgradeFailureDescriptionCn = "ota_upgrade_failure_description_cn";
const char* kOtaUpgradeFailureDescriptionEn = "ota_upgrade_failure_description_en";

/** 未绑定展示/存盘：中文模式用 kUsernameUnboundCn，英文模式用 kUsernameUnboundEn */
constexpr const char* kUsernameUnboundCn = "未绑定";
constexpr const char* kUsernameUnboundEn = "Unbound";

bool isUnboundUsername(const std::string& s) {
    return s.empty() || s == kUsernameUnboundCn || s == kUsernameUnboundEn;
}

const char* unboundStorageForLanguage(Language lang) {
    return (lang == Language::EN) ? kUsernameUnboundEn : kUsernameUnboundCn;
}

std::string getDefaultConfigDir() {
    return "/opt/freego";
}

bool ensureDirExists(const std::string& path) {
    size_t p = path.find_last_of('/');
    if (p == std::string::npos) return true;
    std::string dir = path.substr(0, p);
    if (dir.empty()) return true;
    struct stat st;
    if (stat(dir.c_str(), &st) == 0) return S_ISDIR(st.st_mode);
    size_t pos = 0;
    for (;;) {
        pos = dir.find('/', pos + 1);
        std::string sub = pos == std::string::npos ? dir : dir.substr(0, pos);
        if (sub.empty()) continue;
        if (stat(sub.c_str(), &st) != 0) {
            if (mkdir(sub.c_str(), 0755) != 0) return false;
        } else if (!S_ISDIR(st.st_mode)) {
            return false;
        }
        if (pos == std::string::npos) break;
    }
    return true;
}

}  // namespace

TftUserConfig::TftUserConfig() {
    username_ = unboundStorageForLanguage(language_);
}

TftUserConfig& TftUserConfig::instance() {
    static TftUserConfig inst;
    return inst;
}

std::string TftUserConfig::getConfigPath() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (config_path_.empty()) {
        const_cast<TftUserConfig*>(this)->setConfigPathFromEnv();
    }
    return config_path_;
}

void TftUserConfig::setConfigPathFromEnv() {
    config_path_ = getDefaultConfigDir() + "/user_config.yaml";
}

bool TftUserConfig::load() {
    std::string path;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (config_path_.empty()) setConfigPathFromEnv();
        path = config_path_;
    }
    if (!ensureDirExists(path)) return false;

    std::ifstream f(path);
    if (!f.good()) {
        // 文件不存在：使用默认值并创建配置文件
        return save();
    }

    try {
        YAML::Node node = YAML::Load(f);
        std::lock_guard<std::mutex> lock(mutex_);
        if (node[kUsername]) username_ = node[kUsername].as<std::string>(username_);
        if (node[kVehicleMode]) vehicle_mode_ = node[kVehicleMode].as<int>(vehicle_mode_);
        if (node[kSoundOn]) sound_on_ = node[kSoundOn].as<int>(sound_on_);
        if (node[kSoftwareVersion]) software_version_ = node[kSoftwareVersion].as<std::string>(software_version_);
        if (node[kUpgradeAvailable]) upgrade_available_ = node[kUpgradeAvailable].as<int>(upgrade_available_);
        if (node[kDownloadAvailable]) download_available_ = node[kDownloadAvailable].as<int>(download_available_);
        if (node[kAppDownloadPageState]) app_download_page_state_ = node[kAppDownloadPageState].as<int>(app_download_page_state_);
        if (node[kAppBindPageState]) app_bind_page_state_ = node[kAppBindPageState].as<int>(app_bind_page_state_);
        if (node[kSnCode]) sn_code_ = node[kSnCode].as<std::string>(sn_code_);
        if (node[kBluetoothMac]) bluetooth_mac_ = node[kBluetoothMac].as<std::string>(bluetooth_mac_);
        if (node[kCurrentVersion]) current_version_ = node[kCurrentVersion].as<std::string>(current_version_);
        if (node[kLanguage]) language_ = static_cast<Language>(node[kLanguage].as<int>(static_cast<int>(language_)));
        if (node[kFollowmeEnabled]) followme_enabled_ = node[kFollowmeEnabled].as<int>(followme_enabled_);
        if (node[kSafetyWarningEnabled]) safety_warning_enabled_ = node[kSafetyWarningEnabled].as<int>(safety_warning_enabled_);
        if (node[kGearPosition]) gear_position_ = node[kGearPosition].as<int>(gear_position_);
        if (node[kPrivacyPolicy]) privacy_policy_ = node[kPrivacyPolicy].as<std::string>(privacy_policy_);
        if (node[kOtaUpgradeFailureDescriptionCn]) ota_upgrade_failure_description_cn_ = node[kOtaUpgradeFailureDescriptionCn].as<std::string>(ota_upgrade_failure_description_cn_);
        if (node[kOtaUpgradeFailureDescriptionEn]) ota_upgrade_failure_description_en_ = node[kOtaUpgradeFailureDescriptionEn].as<std::string>(ota_upgrade_failure_description_en_);
        if (isUnboundUsername(username_)) {
            username_ = unboundStorageForLanguage(language_);
        }
        return true;
    } catch (const YAML::Exception&) {
        return false;
    }
}

bool TftUserConfig::save() {
    std::string path;
    std::string u;
    int vm, so;
    std::string ver;
    int ua, da;
    int adps, abps;
    std::string sn, mac;
    std::string cv;
    int lang, fe, swe, gp;
    std::string ota_failure_cn;
    std::string ota_failure_en;
    std::string pp;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (config_path_.empty()) setConfigPathFromEnv();
        path = config_path_;
        u = username_;
        vm = vehicle_mode_;
        so = sound_on_;
        ver = software_version_;
        ua = upgrade_available_;
        da = download_available_;
        adps = app_download_page_state_;
        abps = app_bind_page_state_;
        sn = sn_code_;
        mac = bluetooth_mac_;
        cv = current_version_;
        lang = static_cast<int>(language_);
        fe = followme_enabled_;
        swe = safety_warning_enabled_;
        gp = gear_position_;
        pp = privacy_policy_;
        ota_failure_cn = ota_upgrade_failure_description_cn_;
        ota_failure_en = ota_upgrade_failure_description_en_;
    }
    if (!ensureDirExists(path)) return false;
    try {
        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << kUsername << YAML::Value << u;
        out << YAML::Key << kVehicleMode << YAML::Value << vm;
        out << YAML::Key << kSoundOn << YAML::Value << so;
        out << YAML::Key << kSoftwareVersion << YAML::Value << ver;
        out << YAML::Key << kUpgradeAvailable << YAML::Value << ua;
        out << YAML::Key << kDownloadAvailable << YAML::Value << da;
        out << YAML::Key << kAppDownloadPageState << YAML::Value << adps;
        out << YAML::Key << kAppBindPageState << YAML::Value << abps;
        out << YAML::Key << kSnCode << YAML::Value << sn;
        out << YAML::Key << kBluetoothMac << YAML::Value << mac;
        out << YAML::Key << kCurrentVersion << YAML::Value << cv;
        out << YAML::Key << kLanguage << YAML::Value << lang;
        out << YAML::Key << kFollowmeEnabled << YAML::Value << fe;
        out << YAML::Key << kSafetyWarningEnabled << YAML::Value << swe;
        out << YAML::Key << kGearPosition << YAML::Value << gp;
        out << YAML::Key << kPrivacyPolicy << YAML::Value << pp;
        out << YAML::Key << kOtaUpgradeFailureDescriptionCn << YAML::Value << ota_failure_cn;
        out << YAML::Key << kOtaUpgradeFailureDescriptionEn << YAML::Value << ota_failure_en;
        out << YAML::EndMap;

        std::ofstream f(path);
        if (!f) return false;
        f << out.c_str();
        return true;
    } catch (const YAML::Exception&) {
        return false;
    }
}

std::string TftUserConfig::getUsername() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return username_;
}
void TftUserConfig::setUsername(const std::string& v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (isUnboundUsername(v)) {
            username_ = unboundStorageForLanguage(language_);
        } else {
            username_ = v;
        }
    }
    save();
}

int TftUserConfig::getVehicleMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vehicle_mode_;
}
void TftUserConfig::setVehicleMode(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        vehicle_mode_ = v;
    }
    save();
}

int TftUserConfig::getSoundOn() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sound_on_;
}
void TftUserConfig::setSoundOn(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sound_on_ = v;
    }
    save();
}

Language TftUserConfig::getLanguage() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return language_;
}
void TftUserConfig::setLanguage(Language v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        language_ = v;
        if (isUnboundUsername(username_)) {
            username_ = unboundStorageForLanguage(language_);
        }
    }
    save();
} 

int TftUserConfig::getFollowmeEnabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return followme_enabled_;
}
void TftUserConfig::setFollowmeEnabled(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        followme_enabled_ = v;
    }
    save();
}

std::string TftUserConfig::getSoftwareVersion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return software_version_;
}
void TftUserConfig::setSoftwareVersion(const std::string& v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        software_version_ = v;
    }
    save();
}

int TftUserConfig::getUpgradeAvailable() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return upgrade_available_;
}
void TftUserConfig::setUpgradeAvailable(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        upgrade_available_ = v;
    }
    save();
}

int TftUserConfig::getDownloadAvailable() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return download_available_;
}
void TftUserConfig::setDownloadAvailable(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        download_available_ = v;
    }
    save();
}

int TftUserConfig::getAppDownloadPageState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return app_download_page_state_;
}
void TftUserConfig::setAppDownloadPageState(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        app_download_page_state_ = v;
    }
    save();
}

int TftUserConfig::getAppBindPageState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return app_bind_page_state_;
}
void TftUserConfig::setAppBindPageState(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        app_bind_page_state_ = v;
    }
    save();
}

void TftUserConfig::setSnCode(const std::string& sn) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sn_code_ = sn;
    }
    save();
}
void TftUserConfig::setBluetoothMac(const std::string& bluetooth_mac) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        bluetooth_mac_ = bluetooth_mac;
    }
    save();
}

std::string TftUserConfig::getPrivacyPolicy() {
    std::lock_guard<std::mutex> lock(mutex_);
    return privacy_policy_;
}
void TftUserConfig::setPrivacyPolicy(const std::string& v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        privacy_policy_ = v;
    }
    save();
}

std::string TftUserConfig::getBluetoothMac() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return bluetooth_mac_;
}

std::string TftUserConfig::getAppDownloadCode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return app_download_code_;
}

std::string TftUserConfig::getSnCode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sn_code_;
}

std::string TftUserConfig::getSettingPageBindingQrCode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return sn_code_ + "+" + bluetooth_mac_;
}

std::string TftUserConfig::getCurrentVersion() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_version_;
}
void TftUserConfig::setCurrentVersion(const std::string& v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_version_ = v;
    }
    save();
}

int TftUserConfig::getSafetyWarningEnabled() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return safety_warning_enabled_;
}

void TftUserConfig::setSafetyWarningEnabled(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        safety_warning_enabled_ = v ? 1 : 0;
    }
    save();
}

int TftUserConfig::getGearPosition() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return gear_position_;
}

void TftUserConfig::setGearPosition(int v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (v < 1) {
            gear_position_ = 1;
        } else if (v > 4) {
            gear_position_ = 4;
        } else {
            gear_position_ = v;
        }
    }
    save();
}
bool TftUserConfig::getOtaNewPackageFlag() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ota_new_package_flag_;
}
void TftUserConfig::setOtaNewPackageFlag(const bool& v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ota_new_package_flag_ = v;
    }
}

std::string TftUserConfig::getOtaUpgradeFailureDescriptionCn() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ota_upgrade_failure_description_cn_;
}

std::string TftUserConfig::getOtaUpgradeFailureDescriptionEn() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ota_upgrade_failure_description_en_;
}

std::string TftUserConfig::getOtaUpgradeFailureDescription() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (language_ == Language::EN) {
        return ota_upgrade_failure_description_en_;
    } else {
        return ota_upgrade_failure_description_cn_;
    }
}

void TftUserConfig::setOtaUpgradeFailureDescriptionCn(const std::string& v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ota_upgrade_failure_description_cn_ = v;
    }
    save();
}

void TftUserConfig::setOtaUpgradeFailureDescriptionEn(const std::string& v) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ota_upgrade_failure_description_en_ = v;
    }
    save();
}


}  // namespace tft_service
