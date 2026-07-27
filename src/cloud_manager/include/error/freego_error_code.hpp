#pragma once
#include <cstdint>

namespace freego_error_code {
    enum class ErrorCode {
        // ================ 系统基础错误码 (0x01) ================
        
        // 配置相关错误 (0x010101)
        SYS_CONFIG_ERROR_01 = 0x01010101,  // 配置读取失败 - 类型1
        SYS_CONFIG_ERROR_02 = 0x01010102,  // 配置读取失败 - 类型2
        
        // 初始化相关错误 (0x010102)
        SYS_INIT_FAIL_01 = 0x01010201,     // 初始化失败 - 类型1
        SYS_INIT_FAIL_02 = 0x01010202,     // 初始化失败 - 类型2
        
        // 文件IO错误 (0x010103)
        SYS_FILE_ERROR_01 = 0x01010301,    // 文件IO异常 - 类型1
        SYS_FILE_ERROR_02 = 0x01010302,    // 文件IO异常 - 类型2
        
        // 网络相关错误 (0x010201)
        SYS_NETWORK_FAIL_01 = 0x01020101,  // 4G/网络错误 - 类型1
        SYS_NETWORK_FAIL_02 = 0x01020102,  // 4G/网络错误 - 类型2
        
        // 时间同步错误 (0x010501)
        SYS_TIME_SYNC_ERROR_01 = 0x01050101, // 系统时间异常 - 类型1
        SYS_TIME_SYNC_ERROR_02 = 0x01050102, // 系统时间异常 - 类型2
        
        // ================ IMU错误码 (0x10) ================
        
        // IMU连接错误 (0x100201)
        IMU_NOT_FOUND_01 = 0x10020101,      // 未检测到IMU - 类型1
        IMU_NOT_FOUND_02 = 0x10020102,      // 未检测到IMU - 类型2
        IMU_DISCONNECTED_01 = 0x10020101,   // IMU断连 - 类型1
        IMU_DISCONNECTED_02 = 0x10020102,   // IMU断连 - 类型2
        
        // IMU超时错误 (0x100303)
        IMU_TIMEOUT_01 = 0x10030301,        // 超时无数据 - 类型1
        IMU_TIMEOUT_02 = 0x10030302,        // 超时无数据 - 类型2
        
        // IMU数据错误 (0x100304)
        IMU_DATA_INVALID_01 = 0x10030401,   // 数据异常(NaN/饱和) - 类型1
        IMU_DATA_INVALID_02 = 0x10030402,   // 数据异常(NaN/饱和) - 类型2
        
        // IMU校准错误 (0x100407)
        IMU_CALIB_FAIL_01 = 0x10040701,     // 校准失败 - 类型1
        IMU_CALIB_FAIL_02 = 0x10040702,     // 校准失败 - 类型2
        
        // IMU时间同步错误 (0x100508)
        IMU_TIME_SYNC_ERROR_01 = 0x10050801, // 时间戳跳变 - 类型1
        IMU_TIME_SYNC_ERROR_02 = 0x10050802, // 时间戳跳变 - 类型2
        
        // IMU驱动错误 (0x100609)
        IMU_DRIVER_ERROR_01 = 0x10060901,   // SDK/协议解析失败 - 类型1
        IMU_DRIVER_ERROR_02 = 0x10060902,   // SDK/协议解析失败 - 类型2
        
        // ================ GPS错误码 (0x11) ================
        
        // GPS连接错误 (0x110201)
        GPS_NOT_FOUND_01 = 0x11020101,      // 未检测到GPS - 类型1
        GPS_NOT_FOUND_02 = 0x11020102,      // 未检测到GPS - 类型2
        GPS_DISCONNECTED_01 = 0x11020201,   // 断连 - 类型1
        GPS_DISCONNECTED_02 = 0x11020202,   // 断连 - 类型2
        
        // GPS信号错误 (0x110304)
        GPS_NO_SIGNAL_01 = 0x11030401,      // 无卫星/SNR太低 - 类型1
        GPS_NO_SIGNAL_02 = 0x11030402,      // 无卫星/SNR太低 - 类型2
        
        // GPS数据错误 (0x110305)
        GPS_DATA_LOSS_01 = 0x11030501,      // 数据丢包 - 类型1
        GPS_DATA_LOSS_02 = 0x11030502,      // 数据丢包 - 类型2
        
        // GPS时间同步错误 (0x110508)
        GPS_TIME_SYNC_ERROR_01 = 0x11050801, // PPS/时间异常 - 类型1
        GPS_TIME_SYNC_ERROR_02 = 0x11050802, // PPS/时间异常 - 类型2
        
        // GPS驱动错误 (0x110609)
        GPS_DRIVER_ERROR_01 = 0x11060901,   // 协议解析失败(NMEA/Ublox) - 类型1
        GPS_DRIVER_ERROR_02 = 0x11060902,   // 协议解析失败(NMEA/Ublox) - 类型2
        
        // ================ 单线雷达错误码 (0x12-0x16) ================
        
        // LIDAR1 (左雷达) - 0x12
        LIDAR1_NOT_FOUND_01 = 0x12020101,      // 未检测到雷达 - 类型1
        LIDAR1_NOT_FOUND_02 = 0x12020102,      // 未检测到雷达 - 类型2
        LIDAR1_DISCONNECTED_01 = 0x12020201,   // 断连 - 类型1
        LIDAR1_DISCONNECTED_02 = 0x12020202,   // 断连 - 类型2
        LIDAR1_TIMEOUT_01 = 0x12030301,        // 无数据 - 类型1
        LIDAR1_TIMEOUT_02 = 0x12030302,        // 无数据 - 类型2
        LIDAR1_DATA_INVALID_01 = 0x12030401,   // 数据无效 - 类型1
        LIDAR1_DATA_INVALID_02 = 0x12030402,   // 数据无效 - 类型2
        LIDAR1_TIME_SYNC_ERROR_01 = 0x12050801,// 时间戳异常 - 类型1
        LIDAR1_TIME_SYNC_ERROR_02 = 0x12050802,// 时间戳异常 - 类型2
        LIDAR1_DRIVER_ERROR_01 = 0x12060901,   // 协议解析失败 - 类型1
        LIDAR1_DRIVER_ERROR_02 = 0x12060902,   // 协议解析失败 - 类型2
        
        // LIDAR2 (右雷达) - 0x13
        LIDAR2_NOT_FOUND_01 = 0x13020101,      // 未检测到雷达 - 类型1
        LIDAR2_NOT_FOUND_02 = 0x13020102,      // 未检测到雷达 - 类型2
        LIDAR2_DISCONNECTED_01 = 0x13020201,   // 断连 - 类型1
        LIDAR2_DISCONNECTED_02 = 0x13020202,   // 断连 - 类型2
        LIDAR2_TIMEOUT_01 = 0x13030301,        // 无数据 - 类型1
        LIDAR2_TIMEOUT_02 = 0x13030302,        // 无数据 - 类型2
        LIDAR2_DATA_INVALID_01 = 0x13030401,   // 数据无效 - 类型1
        LIDAR2_DATA_INVALID_02 = 0x13030402,   // 数据无效 - 类型2
        LIDAR2_TIME_SYNC_ERROR_01 = 0x13050801,// 时间戳异常 - 类型1
        LIDAR2_TIME_SYNC_ERROR_02 = 0x13050802,// 时间戳异常 - 类型2
        LIDAR2_DRIVER_ERROR_01 = 0x13060901,   // 协议解析失败 - 类型1
        LIDAR2_DRIVER_ERROR_02 = 0x13060902,   // 协议解析失败 - 类型2
        
        // LIDAR3 (左斜雷达) - 0x14
        LIDAR3_NOT_FOUND_01 = 0x14020101,      // 未检测到雷达 - 类型1
        LIDAR3_NOT_FOUND_02 = 0x14020102,      // 未检测到雷达 - 类型2
        LIDAR3_DISCONNECTED_01 = 0x14020201,   // 断连 - 类型1
        LIDAR3_DISCONNECTED_02 = 0x14020202,   // 断连 - 类型2
        LIDAR3_TIMEOUT_01 = 0x14030301,        // 无数据 - 类型1
        LIDAR3_TIMEOUT_02 = 0x14030302,        // 无数据 - 类型2
        LIDAR3_DATA_INVALID_01 = 0x14030401,   // 数据无效 - 类型1
        LIDAR3_DATA_INVALID_02 = 0x14030402,   // 数据无效 - 类型2
        LIDAR3_TIME_SYNC_ERROR_01 = 0x14050801,// 时间戳异常 - 类型1
        LIDAR3_TIME_SYNC_ERROR_02 = 0x14050802,// 时间戳异常 - 类型2
        LIDAR3_DRIVER_ERROR_01 = 0x14060901,   // 协议解析失败 - 类型1
        LIDAR3_DRIVER_ERROR_02 = 0x14060902,   // 协议解析失败 - 类型2
        
        // LIDAR4 (右斜雷达) - 0x15
        LIDAR4_NOT_FOUND_01 = 0x15020101,      // 未检测到雷达 - 类型1
        LIDAR4_NOT_FOUND_02 = 0x15020102,      // 未检测到雷达 - 类型2
        LIDAR4_DISCONNECTED_01 = 0x15020201,   // 断连 - 类型1
        LIDAR4_DISCONNECTED_02 = 0x15020202,   // 断连 - 类型2
        LIDAR4_TIMEOUT_01 = 0x15030301,        // 无数据 - 类型1
        LIDAR4_TIMEOUT_02 = 0x15030302,        // 无数据 - 类型2
        LIDAR4_DATA_INVALID_01 = 0x15030401,   // 数据无效 - 类型1
        LIDAR4_DATA_INVALID_02 = 0x15030402,   // 数据无效 - 类型2
        LIDAR4_TIME_SYNC_ERROR_01 = 0x15050801,// 时间戳异常 - 类型1
        LIDAR4_TIME_SYNC_ERROR_02 = 0x15050802,// 时间戳异常 - 类型2
        LIDAR4_DRIVER_ERROR_01 = 0x15060901,   // 协议解析失败 - 类型1
        LIDAR4_DRIVER_ERROR_02 = 0x15060902,   // 协议解析失败 - 类型2
        
        // LIDAR5 (后雷达) - 0x16
        LIDAR5_NOT_FOUND_01 = 0x16020101,      // 未检测到雷达 - 类型1
        LIDAR5_NOT_FOUND_02 = 0x16020102,      // 未检测到雷达 - 类型2
        LIDAR5_DISCONNECTED_01 = 0x16020201,   // 断连 - 类型1
        LIDAR5_DISCONNECTED_02 = 0x16020202,   // 断连 - 类型2
        LIDAR5_TIMEOUT_01 = 0x16030301,        // 无数据 - 类型1
        LIDAR5_TIMEOUT_02 = 0x16030302,        // 无数据 - 类型2
        LIDAR5_DATA_INVALID_01 = 0x16030401,   // 数据无效 - 类型1
        LIDAR5_DATA_INVALID_02 = 0x16030402,   // 数据无效 - 类型2
        LIDAR5_TIME_SYNC_ERROR_01 = 0x16050801,// 时间戳异常 - 类型1
        LIDAR5_TIME_SYNC_ERROR_02 = 0x16050802,// 时间戳异常 - 类型2
        LIDAR5_DRIVER_ERROR_01 = 0x16060901,   // 协议解析失败 - 类型1
        LIDAR5_DRIVER_ERROR_02 = 0x16060902,   // 协议解析失败 - 类型2
        
        // ================ 相机错误码 (0x17-0x18) ================
        
        // CAM1 (RGB单目) - 0x17
        CAM1_NOT_FOUND_01 = 0x17020101,        // 未检测到相机 - 类型1
        CAM1_NOT_FOUND_02 = 0x17020102,        // 未检测到相机 - 类型2
        CAM1_DISCONNECTED_01 = 0x17020201,     // 断连 - 类型1
        CAM1_DISCONNECTED_02 = 0x17020202,     // 断连 - 类型2
        CAM1_DISCONNECTED_TOO_MUCH_01 = 0x17030301, // 断连过多 - 类型1
        CAM1_DISCONNECTED_TOO_MUCH_02 = 0x17030302, // 断连过多 - 类型2
        CAM1_FRAME_DROP_01 = 0x17030501,       // 丢帧过多 - 类型1
        CAM1_FRAME_DROP_02 = 0x17030502,       // 丢帧过多 - 类型2
        CAM1_DATA_INVALID_01 = 0x17030401,     // 图像损坏/格式错误 - 类型1
        CAM1_DATA_INVALID_02 = 0x17030402,     // 图像损坏/格式错误 - 类型2
        CAM1_TIME_SYNC_ERROR_01 = 0x17050801,  // 时间异常 - 类型1
        CAM1_TIME_SYNC_ERROR_02 = 0x17050802,  // 时间异常 - 类型2
        CAM1_DRIVER_ERROR_01 = 0x17060901,     // SDK/驱动错误 - 类型1
        CAM1_DRIVER_ERROR_02 = 0x17060902,     // SDK/驱动错误 - 类型2
        
        // CAM2 (RGB-D) - 0x18
        CAM2_NOT_FOUND_01 = 0x18020101,        // 未检测到相机 - 类型1
        CAM2_NOT_FOUND_02 = 0x18020102,        // 未检测到相机 - 类型2
        CAM2_DISCONNECTED_01 = 0x18020201,     // 断连 - 类型1
        CAM2_DISCONNECTED_02 = 0x18020202,     // 断连 - 类型2
        CAM2_DEPTH_INVALID_01 = 0x18030401,    // 丢帧过多 - 类型1
        CAM2_DEPTH_INVALID_02 = 0x18030402,    // 丢帧过多 - 类型2
        CAM2_FRAME_DROP_01 = 0x18030501,       // 图像损坏/格式错误 - 类型1
        CAM2_FRAME_DROP_02 = 0x18030502,       // 图像损坏/格式错误 - 类型2
        CAM2_TIME_SYNC_ERROR_01 = 0x18050801,  // 时间异常 - 类型1
        CAM2_TIME_SYNC_ERROR_02 = 0x18050802,  // 时间异常 - 类型2
        CAM2_DRIVER_ERROR_01 = 0x18060901,     // SDK/驱动错误 - 类型1
        CAM2_DRIVER_ERROR_02 = 0x18060902,     // SDK/驱动错误 - 类型2
        
        // ================ TFT错误码 (0x19) ================
        
        TFT_NOT_FOUND_01 = 0x19020101,         // 未检测到TFT - 类型1
        TFT_NOT_FOUND_02 = 0x19020102,         // 未检测到TFT - 类型2
        TFT_DISCONNECTED_01 = 0x19020201,      // 断连 - 类型1
        TFT_DISCONNECTED_02 = 0x19020202,      // 断连 - 类型2
        TFT_DEPTH_INVALID_01 = 0x19030401,     // 暂未定 - 类型1
        TFT_DEPTH_INVALID_02 = 0x19030402,     // 暂未定 - 类型2
        TFT_FRAME_DROP_01 = 0x19030501,        // 暂未定 - 类型1
        TFT_FRAME_DROP_02 = 0x19030502,        // 暂未定 - 类型2
        TFT_TIME_SYNC_ERROR_01 = 0x19050801,   // 暂未定 - 类型1
        TFT_TIME_SYNC_ERROR_02 = 0x19050802,   // 暂未定 - 类型2
        TFT_DRIVER_ERROR_01 = 0x19060901,      // 暂未定 - 类型1
        TFT_DRIVER_ERROR_02 = 0x19060902,      // 暂未定 - 类型2
        
        // ================ CAN错误码 (0x02-0x04) ================
        
        // CAN0 - 0x02
        CAN0_NOT_FOUND_01 = 0x02020101,        // 未检测到CAN0 - 类型1
        CAN0_NOT_FOUND_02 = 0x02020102,        // 未检测到CAN0 - 类型2
        CAN0_DISCONNECTED_01 = 0x02020201,     // 断连 - 类型1
        CAN0_DISCONNECTED_02 = 0x02020202,     // 断连 - 类型2
        
        // CAN1 - 0x03
        CAN1_NOT_FOUND_01 = 0x03020101,        // 未检测到CAN1 - 类型1
        CAN1_NOT_FOUND_02 = 0x03020102,        // 未检测到CAN1 - 类型2
        CAN1_DISCONNECTED_01 = 0x03020201,     // 断连 - 类型1
        CAN1_DISCONNECTED_02 = 0x03020202,     // 断连 - 类型2
        
        // CAN2 - 0x04
        CAN2_NOT_FOUND_01 = 0x04020101,        // 未检测到CAN2 - 类型1
        CAN2_NOT_FOUND_02 = 0x04020102,        // 未检测到CAN2 - 类型2
        CAN2_DISCONNECTED_01 = 0x04020201,     // 断连 - 类型1
        CAN2_DISCONNECTED_02 = 0x04020202,     // 断连 - 类型2
        // ================ 电机驱动器错误码 (DRV = 0x20) ================
        
        ERROR_AXIS0_01 = 0x20070101,            // 错误无效状态 - 类型1
        ERROR_AXIS0_02 = 0x20070102,            // 错误无效状态 - 类型2
        ERROR_AXIS1_01 = 0x20080101,            // 直流母线欠压错误 - 类型1
        ERROR_AXIS1_02 = 0x20080102,            // 直流母线欠压错误 - 类型2
        ERROR_BRAKE_DISABLE_01 = 0x20020101,    // 抱闸失效 - 类型1
        ERROR_BRAKE_DISABLE_02 = 0x20020102,    // 抱闸失效 - 类型2
        
        // ================ 上控错误码 (TOP = 0x21) ================
        
        ERROR_INVALID_STATE_01 = 0x21010101,    // 错误无效状态 - 类型1
        ERROR_INVALID_STATE_02 = 0x21010102,    // 错误无效状态 - 类型2
        ERROR_JOYSTICK_01 = 0x21010201,         // 摇杆失效 - 类型1
        ERROR_JOYSTICK_02 = 0x21010202,         // 摇杆失效 - 类型2
        ERROR_CAN_COM_01 = 0x21020301,          // 电机驱动器断连 - 类型1
        ERROR_CAN_COM_02 = 0x21020302,          // 电机驱动器断连 - 类型2
        
        // ================ 功能故障码 (0x05-0x09) ================
        
        // 避障功能异常 - 0x05
        ERROR_AUX_01 = 0x05010101,              // 安全功能异常 - 类型1
        ERROR_AUX_02 = 0x05010102,              // 安全功能异常 - 类型2
        
        // AI巡航功能异常 - 0x06
        ERROR_CRU_01 = 0x06010101,              // AI巡航功能异常 - 类型1
        ERROR_CRU_02 = 0x06010102,              // AI巡航功能异常 - 类型2
        
        // 倒车辅助功能异常 - 0x07
        ERROR_RFH_01 = 0x07010101,              // 倒车辅助功能异常 - 类型1
        ERROR_RFH_02 = 0x07010102,              // 倒车辅助功能异常 - 类型2
        
        // 回充功能异常 - 0x08
        ERROR_REC_01 = 0x08010101,              // 回充功能异常 - 类型1
        ERROR_REC_02 = 0x08010102,              // 回充功能异常 - 类型2
        ERROR_REC_NODOCK_01 = 0x08010201,       // 未找到充电桩 - 类型1
        ERROR_REC_NODOCK_02 = 0x08010202,       // 未找到充电桩 - 类型2
        ERROR_REC_BLOCK_01 = 0x08010301,        // 阻塞卡住了 - 类型1
        ERROR_REC_BLOCK_02 = 0x08010302,        // 阻塞卡住了 - 类型2
        ERROR_REC_NAV_INIT_01 = 0x08010401,     // 回充导航无法初始化 - 类型1
        ERROR_REC_NAV_INIT_02 = 0x08010402,     // 回充导航无法初始化 - 类型2
        ERROR_REC_NAV_UNREACHABLE_01 = 0x08010501, // 回充导航无法到达终点 - 类型1
        ERROR_REC_NAV_UNREACHABLE_02 = 0x08010502, // 回充导航无法到达终点 - 类型2
        ERROR_REC_NAV_TIMEOUT_01 = 0x08010601,  // 回充导航超时 - 类型1
        ERROR_REC_NAV_TIMEOUT_02 = 0x08010602,  // 回充导航超时 - 类型2
        ERROR_REC_NAV_ABORT_01 = 0x08010701,    // 回充导航异常退出 - 类型1
        ERROR_REC_NAV_ABORT_02 = 0x08010702,    // 回充导航异常退出 - 类型2
        ERROR_REC_VOLTAGE_ABNORMAL_01 = 0x08010801, // 电压异常 - 类型1
        ERROR_REC_VOLTAGE_ABNORMAL_02 = 0x08010802, // 电压异常 - 类型2
        ERROR_REC_OBSTACLE_01 = 0x08010901,     // 障碍物 - 类型1
        ERROR_REC_OBSTACLE_02 = 0x08010902,     // 障碍物 - 类型2
        ERROR_REC_SENSOR_ERROR_01 = 0x08010A01, // 传感器异常 - 类型1
        ERROR_REC_SENSOR_ERROR_02 = 0x08010A02, // 传感器异常 - 类型2
        
        // 推行功能异常 - 0x09
        ERROR_PUSH_01 = 0x09010101,             // 推行功能异常 - 类型1
        ERROR_PUSH_02 = 0x09010102,             // 推行功能异常 - 类型2

        // ================ 未知错误码 (0x0A) ================
        UNKNOWN_ERROR = 0x0A000000,            // 未知错误码 - 类型1
    };

    // 错误码信息结构
    typedef struct {
        uint8_t status;        // 生效或解除
        uint8_t count;         // 累计次数
        bool reported;         // 是否已上报
    } ErrorInfo;
}