#pragma once

#include "freego_error_code.hpp"
#include <string>
#include <unordered_map>

namespace freego_error_code {

/**
 * @brief 获取错误码的描述信息
 * @param code 错误码
 * @return 错误码描述字符串
 */
inline std::string get_error_description(ErrorCode code) {
    static const std::unordered_map<ErrorCode, std::string> error_descriptions = {
        // 配置相关错误
        {ErrorCode::SYS_CONFIG_ERROR_01, "配置读取失败"},
        {ErrorCode::SYS_CONFIG_ERROR_02, "配置读取失败"},
        
        // 初始化相关错误
        {ErrorCode::SYS_INIT_FAIL_01, "初始化失败"},
        {ErrorCode::SYS_INIT_FAIL_02, "初始化失败"},
        
        // 文件IO错误
        {ErrorCode::SYS_FILE_ERROR_01, "文件IO异常"},
        {ErrorCode::SYS_FILE_ERROR_02, "文件IO异常"},
        
        // 网络相关错误
        {ErrorCode::SYS_NETWORK_FAIL_01, "4G/网络错误"},
        {ErrorCode::SYS_NETWORK_FAIL_02, "4G/网络错误"},
        
        // 时间同步错误
        {ErrorCode::SYS_TIME_SYNC_ERROR_01, "系统时间异常"},
        {ErrorCode::SYS_TIME_SYNC_ERROR_02, "系统时间异常"},
        
        // IMU连接错误
        {ErrorCode::IMU_NOT_FOUND_01, "未检测到IMU"},
        {ErrorCode::IMU_NOT_FOUND_02, "未检测到IMU"},
        {ErrorCode::IMU_DISCONNECTED_01, "IMU断连"},
        {ErrorCode::IMU_DISCONNECTED_02, "IMU断连"},
        
        // IMU超时错误
        {ErrorCode::IMU_TIMEOUT_01, "超时无数据"},
        {ErrorCode::IMU_TIMEOUT_02, "超时无数据"},
        
        // IMU数据错误
        {ErrorCode::IMU_DATA_INVALID_01, "数据异常(NaN/饱和)"},
        {ErrorCode::IMU_DATA_INVALID_02, "数据异常(NaN/饱和)"},
        
        // IMU校准错误
        {ErrorCode::IMU_CALIB_FAIL_01, "校准失败"},
        {ErrorCode::IMU_CALIB_FAIL_02, "校准失败"},
        
        // IMU时间同步错误
        {ErrorCode::IMU_TIME_SYNC_ERROR_01, "时间戳跳变"},
        {ErrorCode::IMU_TIME_SYNC_ERROR_02, "时间戳跳变"},
        
        // IMU驱动错误
        {ErrorCode::IMU_DRIVER_ERROR_01, "SDK/协议解析失败"},
        {ErrorCode::IMU_DRIVER_ERROR_02, "SDK/协议解析失败"},
        
        // GPS连接错误
        {ErrorCode::GPS_NOT_FOUND_01, "未检测到GPS"},
        {ErrorCode::GPS_NOT_FOUND_02, "未检测到GPS"},
        {ErrorCode::GPS_DISCONNECTED_01, "断连"},
        {ErrorCode::GPS_DISCONNECTED_02, "断连"},
        
        // GPS信号错误
        {ErrorCode::GPS_NO_SIGNAL_01, "无卫星/SNR太低"},
        {ErrorCode::GPS_NO_SIGNAL_02, "无卫星/SNR太低"},
        
        // GPS数据错误
        {ErrorCode::GPS_DATA_LOSS_01, "数据丢包"},
        {ErrorCode::GPS_DATA_LOSS_02, "数据丢包"},
        
        // GPS时间同步错误
        {ErrorCode::GPS_TIME_SYNC_ERROR_01, "PPS/时间异常"},
        {ErrorCode::GPS_TIME_SYNC_ERROR_02, "PPS/时间异常"},
        
        // GPS驱动错误
        {ErrorCode::GPS_DRIVER_ERROR_01, "协议解析失败(NMEA/Ublox)"},
        {ErrorCode::GPS_DRIVER_ERROR_02, "协议解析失败(NMEA/Ublox)"},
        
        // LIDAR1错误
        {ErrorCode::LIDAR1_NOT_FOUND_01, "未检测到雷达"},
        {ErrorCode::LIDAR1_NOT_FOUND_02, "未检测到雷达"},
        {ErrorCode::LIDAR1_DISCONNECTED_01, "断连"},
        {ErrorCode::LIDAR1_DISCONNECTED_02, "断连"},
        {ErrorCode::LIDAR1_TIMEOUT_01, "无数据"},
        {ErrorCode::LIDAR1_TIMEOUT_02, "无数据"},
        {ErrorCode::LIDAR1_DATA_INVALID_01, "数据无效"},
        {ErrorCode::LIDAR1_DATA_INVALID_02, "数据无效"},
        {ErrorCode::LIDAR1_TIME_SYNC_ERROR_01, "时间戳异常"},
        {ErrorCode::LIDAR1_TIME_SYNC_ERROR_02, "时间戳异常"},
        {ErrorCode::LIDAR1_DRIVER_ERROR_01, "协议解析失败"},
        {ErrorCode::LIDAR1_DRIVER_ERROR_02, "协议解析失败"},
        
        // LIDAR2错误
        {ErrorCode::LIDAR2_NOT_FOUND_01, "未检测到雷达"},
        {ErrorCode::LIDAR2_NOT_FOUND_02, "未检测到雷达"},
        {ErrorCode::LIDAR2_DISCONNECTED_01, "断连"},
        {ErrorCode::LIDAR2_DISCONNECTED_02, "断连"},
        {ErrorCode::LIDAR2_TIMEOUT_01, "无数据"},
        {ErrorCode::LIDAR2_TIMEOUT_02, "无数据"},
        {ErrorCode::LIDAR2_DATA_INVALID_01, "数据无效"},
        {ErrorCode::LIDAR2_DATA_INVALID_02, "数据无效"},
        {ErrorCode::LIDAR2_TIME_SYNC_ERROR_01, "时间戳异常"},
        {ErrorCode::LIDAR2_TIME_SYNC_ERROR_02, "时间戳异常"},
        {ErrorCode::LIDAR2_DRIVER_ERROR_01, "协议解析失败"},
        {ErrorCode::LIDAR2_DRIVER_ERROR_02, "协议解析失败"},
        
        // LIDAR3错误
        {ErrorCode::LIDAR3_NOT_FOUND_01, "未检测到雷达"},
        {ErrorCode::LIDAR3_NOT_FOUND_02, "未检测到雷达"},
        {ErrorCode::LIDAR3_DISCONNECTED_01, "断连"},
        {ErrorCode::LIDAR3_DISCONNECTED_02, "断连"},
        {ErrorCode::LIDAR3_TIMEOUT_01, "无数据"},
        {ErrorCode::LIDAR3_TIMEOUT_02, "无数据"},
        {ErrorCode::LIDAR3_DATA_INVALID_01, "数据无效"},
        {ErrorCode::LIDAR3_DATA_INVALID_02, "数据无效"},
        {ErrorCode::LIDAR3_TIME_SYNC_ERROR_01, "时间戳异常"},
        {ErrorCode::LIDAR3_TIME_SYNC_ERROR_02, "时间戳异常"},
        {ErrorCode::LIDAR3_DRIVER_ERROR_01, "协议解析失败"},
        {ErrorCode::LIDAR3_DRIVER_ERROR_02, "协议解析失败"},
        
        // LIDAR4错误
        {ErrorCode::LIDAR4_NOT_FOUND_01, "未检测到雷达"},
        {ErrorCode::LIDAR4_NOT_FOUND_02, "未检测到雷达"},
        {ErrorCode::LIDAR4_DISCONNECTED_01, "断连"},
        {ErrorCode::LIDAR4_DISCONNECTED_02, "断连"},
        {ErrorCode::LIDAR4_TIMEOUT_01, "无数据"},
        {ErrorCode::LIDAR4_TIMEOUT_02, "无数据"},
        {ErrorCode::LIDAR4_DATA_INVALID_01, "数据无效"},
        {ErrorCode::LIDAR4_DATA_INVALID_02, "数据无效"},
        {ErrorCode::LIDAR4_TIME_SYNC_ERROR_01, "时间戳异常"},
        {ErrorCode::LIDAR4_TIME_SYNC_ERROR_02, "时间戳异常"},
        {ErrorCode::LIDAR4_DRIVER_ERROR_01, "协议解析失败"},
        {ErrorCode::LIDAR4_DRIVER_ERROR_02, "协议解析失败"},
        
        // LIDAR5错误
        {ErrorCode::LIDAR5_NOT_FOUND_01, "未检测到雷达"},
        {ErrorCode::LIDAR5_NOT_FOUND_02, "未检测到雷达"},
        {ErrorCode::LIDAR5_DISCONNECTED_01, "断连"},
        {ErrorCode::LIDAR5_DISCONNECTED_02, "断连"},
        {ErrorCode::LIDAR5_TIMEOUT_01, "无数据"},
        {ErrorCode::LIDAR5_TIMEOUT_02, "无数据"},
        {ErrorCode::LIDAR5_DATA_INVALID_01, "数据无效"},
        {ErrorCode::LIDAR5_DATA_INVALID_02, "数据无效"},
        {ErrorCode::LIDAR5_TIME_SYNC_ERROR_01, "时间戳异常"},
        {ErrorCode::LIDAR5_TIME_SYNC_ERROR_02, "时间戳异常"},
        {ErrorCode::LIDAR5_DRIVER_ERROR_01, "协议解析失败"},
        {ErrorCode::LIDAR5_DRIVER_ERROR_02, "协议解析失败"},
        
        // CAM1错误
        {ErrorCode::CAM1_NOT_FOUND_01, "未检测到相机"},
        {ErrorCode::CAM1_NOT_FOUND_02, "未检测到相机"},
        {ErrorCode::CAM1_DISCONNECTED_01, "断连"},
        {ErrorCode::CAM1_DISCONNECTED_02, "断连"},
        {ErrorCode::CAM1_DISCONNECTED_TOO_MUCH_01, "断连过多"},
        {ErrorCode::CAM1_DISCONNECTED_TOO_MUCH_02, "断连过多"},
        {ErrorCode::CAM1_FRAME_DROP_01, "丢帧过多"},
        {ErrorCode::CAM1_FRAME_DROP_02, "丢帧过多"},
        {ErrorCode::CAM1_DATA_INVALID_01, "图像损坏/格式错误"},
        {ErrorCode::CAM1_DATA_INVALID_02, "图像损坏/格式错误"},
        {ErrorCode::CAM1_TIME_SYNC_ERROR_01, "时间异常"},
        {ErrorCode::CAM1_TIME_SYNC_ERROR_02, "时间异常"},
        {ErrorCode::CAM1_DRIVER_ERROR_01, "SDK/驱动错误"},
        {ErrorCode::CAM1_DRIVER_ERROR_02, "SDK/驱动错误"},
        
        // CAM2错误
        {ErrorCode::CAM2_NOT_FOUND_01, "未检测到相机"},
        {ErrorCode::CAM2_NOT_FOUND_02, "未检测到相机"},
        {ErrorCode::CAM2_DISCONNECTED_01, "断连"},
        {ErrorCode::CAM2_DISCONNECTED_02, "断连"},
        {ErrorCode::CAM2_DEPTH_INVALID_01, "丢帧过多"},
        {ErrorCode::CAM2_DEPTH_INVALID_02, "丢帧过多"},
        {ErrorCode::CAM2_FRAME_DROP_01, "图像损坏/格式错误"},
        {ErrorCode::CAM2_FRAME_DROP_02, "图像损坏/格式错误"},
        {ErrorCode::CAM2_TIME_SYNC_ERROR_01, "时间异常"},
        {ErrorCode::CAM2_TIME_SYNC_ERROR_02, "时间异常"},
        {ErrorCode::CAM2_DRIVER_ERROR_01, "SDK/驱动错误"},
        {ErrorCode::CAM2_DRIVER_ERROR_02, "SDK/驱动错误"},
        
        // TFT错误
        {ErrorCode::TFT_NOT_FOUND_01, "未检测到TFT"},
        {ErrorCode::TFT_NOT_FOUND_02, "未检测到TFT"},
        {ErrorCode::TFT_DISCONNECTED_01, "断连"},
        {ErrorCode::TFT_DISCONNECTED_02, "断连"},
        {ErrorCode::TFT_DEPTH_INVALID_01, "暂未定"},
        {ErrorCode::TFT_DEPTH_INVALID_02, "暂未定"},
        {ErrorCode::TFT_FRAME_DROP_01, "暂未定"},
        {ErrorCode::TFT_FRAME_DROP_02, "暂未定"},
        {ErrorCode::TFT_TIME_SYNC_ERROR_01, "暂未定"},
        {ErrorCode::TFT_TIME_SYNC_ERROR_02, "暂未定"},
        {ErrorCode::TFT_DRIVER_ERROR_01, "暂未定"},
        {ErrorCode::TFT_DRIVER_ERROR_02, "暂未定"},
        
        // CAN0错误
        {ErrorCode::CAN0_NOT_FOUND_01, "未检测到CAN0"},
        {ErrorCode::CAN0_NOT_FOUND_02, "未检测到CAN0"},
        {ErrorCode::CAN0_DISCONNECTED_01, "断连"},
        {ErrorCode::CAN0_DISCONNECTED_02, "断连"},
        
        // CAN1错误
        {ErrorCode::CAN1_NOT_FOUND_01, "未检测到CAN1"},
        {ErrorCode::CAN1_NOT_FOUND_02, "未检测到CAN1"},
        {ErrorCode::CAN1_DISCONNECTED_01, "断连"},
        {ErrorCode::CAN1_DISCONNECTED_02, "断连"},
        
        // CAN2错误
        {ErrorCode::CAN2_NOT_FOUND_01, "未检测到CAN2"},
        {ErrorCode::CAN2_NOT_FOUND_02, "未检测到CAN2"},
        {ErrorCode::CAN2_DISCONNECTED_01, "断连"},
        {ErrorCode::CAN2_DISCONNECTED_02, "断连"},
                // 电机驱动器错误
        {ErrorCode::ERROR_AXIS0_01, "错误无效状态"},
        {ErrorCode::ERROR_AXIS0_02, "错误无效状态"},
        {ErrorCode::ERROR_AXIS1_01, "直流母线欠压错误"},
        {ErrorCode::ERROR_AXIS1_02, "直流母线欠压错误"},
        {ErrorCode::ERROR_BRAKE_DISABLE_01, "抱闸失效"},
        {ErrorCode::ERROR_BRAKE_DISABLE_02, "抱闸失效"},
        
        // 上控错误
        {ErrorCode::ERROR_INVALID_STATE_01, "错误无效状态"},
        {ErrorCode::ERROR_INVALID_STATE_02, "错误无效状态"},
        {ErrorCode::ERROR_JOYSTICK_01, "摇杆失效"},
        {ErrorCode::ERROR_JOYSTICK_02, "摇杆失效"},
        {ErrorCode::ERROR_CAN_COM_01, "电机驱动器断连"},
        {ErrorCode::ERROR_CAN_COM_02, "电机驱动器断连"},
        
        // 避障功能异常
        {ErrorCode::ERROR_AUX_01, "安全功能异常"},
        {ErrorCode::ERROR_AUX_02, "安全功能异常"},
        
        // AI巡航功能异常
        {ErrorCode::ERROR_CRU_01, "AI巡航功能异常"},
        {ErrorCode::ERROR_CRU_02, "AI巡航功能异常"},
        
        // 倒车辅助功能异常
        {ErrorCode::ERROR_RFH_01, "倒车辅助功能异常"},
        {ErrorCode::ERROR_RFH_02, "倒车辅助功能异常"},
        
        // 回充功能异常
        {ErrorCode::ERROR_REC_01, "回充功能异常"},
        {ErrorCode::ERROR_REC_02, "回充功能异常"},
        {ErrorCode::ERROR_REC_NODOCK_01, "未找到充电桩"},
        {ErrorCode::ERROR_REC_NODOCK_02, "未找到充电桩"},
        {ErrorCode::ERROR_REC_BLOCK_01, "阻塞卡住了"},
        {ErrorCode::ERROR_REC_BLOCK_02, "阻塞卡住了"},
        {ErrorCode::ERROR_REC_NAV_INIT_01, "回充导航无法初始化"},
        {ErrorCode::ERROR_REC_NAV_INIT_02, "回充导航无法初始化"},
        {ErrorCode::ERROR_REC_NAV_UNREACHABLE_01, "回充导航无法到达终点"},
        {ErrorCode::ERROR_REC_NAV_UNREACHABLE_02, "回充导航无法到达终点"},
        {ErrorCode::ERROR_REC_NAV_TIMEOUT_01, "回充导航超时"},
        {ErrorCode::ERROR_REC_NAV_TIMEOUT_02, "回充导航超时"},
        {ErrorCode::ERROR_REC_NAV_ABORT_01, "回充导航异常退出"},
        {ErrorCode::ERROR_REC_NAV_ABORT_02, "回充导航异常退出"},
        {ErrorCode::ERROR_REC_VOLTAGE_ABNORMAL_01, "电压异常"},
        {ErrorCode::ERROR_REC_VOLTAGE_ABNORMAL_02, "电压异常"},
        {ErrorCode::ERROR_REC_OBSTACLE_01, "障碍物"},
        {ErrorCode::ERROR_REC_OBSTACLE_02, "障碍物"},
        {ErrorCode::ERROR_REC_SENSOR_ERROR_01, "传感器异常"},
        {ErrorCode::ERROR_REC_SENSOR_ERROR_02, "传感器异常"},
        
        // 推行功能异常
        {ErrorCode::ERROR_PUSH_01, "推行功能异常"},
        {ErrorCode::ERROR_PUSH_02, "推行功能异常"},
        

    };
    
    auto it = error_descriptions.find(code);
    if (it != error_descriptions.end()) {
        return it->second;
    }
    return "未知错误码";
}

/**
 * @brief 获取错误码的模块分类
 * @param code 错误码
 * @return 模块分类字符串
 */
inline std::string get_error_module(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    
    switch (module) {
        case 0x01: return "系统基础模块";
        case 0x02: return "CAN0模块";
        case 0x03: return "CAN1模块";
        case 0x04: return "CAN2模块";
        case 0x05: return "避障功能模块";
        case 0x06: return "AI巡航功能模块";
        case 0x07: return "倒车辅助功能模块";
        case 0x08: return "回充功能模块";
        case 0x09: return "推行功能模块";
        case 0x10: return "IMU模块";
        case 0x11: return "GPS模块";
        case 0x12: return "LIDAR1模块(左雷达)";
        case 0x13: return "LIDAR2模块(右雷达)";
        case 0x14: return "LIDAR3模块(左斜雷达)";
        case 0x15: return "LIDAR4模块(右斜雷达)";
        case 0x16: return "LIDAR5模块(后雷达)";
        case 0x17: return "CAM1模块(RGB单目)";
        case 0x18: return "CAM2模块(RGB-D)";
        case 0x19: return "TFT模块";
        case 0x20: return "电机驱动器模块(DRV)";
        case 0x21: return "上控模块(TOP)";
        default: return "未知模块";
    }
}

/**
 * @brief 将错误码转换为十六进制字符串
 * @param code 错误码
 * @return 十六进制字符串，如 "0x01010101"
 */
inline std::string error_code_to_hex_string(ErrorCode code) {
    char buf[12];
    snprintf(buf, sizeof(buf), "0x%08X", static_cast<uint32_t>(code));
    return std::string(buf);
}

/**
 * @brief 检查错误码是否为系统基础错误
 * @param code 错误码
 * @return true 如果是系统基础错误
 */
inline bool is_system_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return module == 0x01;
}

/**
 * @brief 检查错误码是否为IMU模块错误
 * @param code 错误码
 * @return true 如果是IMU模块错误
 */
inline bool is_imu_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return module == 0x10;
}

/**
 * @brief 判断是否为GPS错误
 * @param code 错误码
 * @return true表示是GPS错误
 */
inline bool is_gps_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return module == 0x11;
}

/**
 * @brief 判断是否为LIDAR错误（任意雷达）
 * @param code 错误码
 * @return true表示是LIDAR错误
 */
inline bool is_lidar_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return (module >= 0x12 && module <= 0x16);
}

/**
 * @brief 判断是否为相机错误
 * @param code 错误码
 * @return true表示是相机错误
 */
inline bool is_camera_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return (module == 0x17 || module == 0x18);
}

/**
 * @brief 判断是否为TFT错误
 * @param code 错误码
 * @return true表示是TFT错误
 */
inline bool is_tft_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return module == 0x19;
}

/**
 * @brief 判断是否为CAN错误（任意CAN通道）
 * @param code 错误码
 * @return true表示是CAN错误
 */
inline bool is_can_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return (module >= 0x02 && module <= 0x04);
}

/**
 * @brief 判断是否为功能故障码（避障/巡航/倒车/回充/推行）
 * @param code 错误码
 * @return true表示是功能故障码
 */
inline bool is_function_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return (module >= 0x05 && module <= 0x09);
}

/**
 * @brief 判断是否为电机驱动器错误
 * @param code 错误码
 * @return true表示是电机驱动器错误
 */
inline bool is_drv_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return module == 0x20;
}

/**
 * @brief 判断是否为上控错误
 * @param code 错误码
 * @return true表示是上控错误
 */
inline bool is_top_error(ErrorCode code) {
    uint32_t value = static_cast<uint32_t>(code);
    uint8_t module = (value >> 24) & 0xFF;
    return module == 0x21;
}
/**
 * @brief 分析错误类型
 * @param code 错误码
 * @return 取前6位作为错误码类型
 */
inline u_int32_t get_error_type(u_int32_t& code) { 

    return (code & 0xFFFFFF00);
}

/**
 * @brief 分析错误状态
 * @param code 错误码
 * @return 取后2位作为错误码类型
 */
inline uint8_t get_error_status(u_int32_t& code) { 

    return (code & 0xFF);
}

} // namespace freego