#pragma once

/**
0x01	系统基础	SYS
0x02	can0	CAN0
0x03	can1	CAN1
0x04	can2	CAN2
0x10	IMU	IMU
0x11	GPS	GPS
0x12	单线雷达 #1 左	LIDAR1
0x13	单线雷达 #2 右	LIDAR2
0x14	单线雷达 #3 左斜	LIDAR3
0x15	单线雷达 #4 右斜	LIDAR4
0x16	单线雷达 #5 后	LIDAR5
0x17	相机 #1（RGB）	CAM1
0x18	相机 #2（RGB-D/后）	CAM2
0x19	屏幕	TFT

0x20	电机驱动器	MOTOR
0x21	上控	TOP
 */
enum class ErrorEventCategory {
    SYS = 0x01,
    CAN0 = 0x02,
    CAN1 = 0x03,
    CAN2 = 0x04,
    IMU = 0x10,
    GPS = 0x11,
    LIDAR1 = 0x12,
    LIDAR2 = 0x13,
    LIDAR3 = 0x14,
    LIDAR4 = 0x15,
    LIDAR5 = 0x16,
    CAM1 = 0x17,
    CAM2 = 0x18,
    TFT = 0x19,
    MOTOR = 0x20,
    TOP = 0x21,
};
//  01 发生  02 恢复
enum class ErrorEventLevel {
    OCCUR = 0x01,
    RECOVER = 0x02,
};

#define SYS_CONFIG_ERROR 0x010101 // 配置读取失败
#define SYS_INIT_FAIL 0x010102 // 初始化失败
#define SYS_FILE_ERROR 0x010103 // 文件 IO 异常
#define SYS_NETWORK_FAIL 0x010201 // 4G / 网络错误
#define SYS_TIME_SYNC_ERROR 0x010501 // 系统时间异常

#define CAN0_NOT_FOUND 0x020201 // 未检测到 CAN0
#define CAN0_DISCONNECTED 0x020202 // CAN0 断连

#define CAN1_NOT_FOUND 0x030201 // 未检测到 CAN1
#define CAN1_DISCONNECTED 0x030202 // CAN1 断连

#define CAN2_NOT_FOUND 0x040201 // 未检测到 CAN2
#define CAN2_DISCONNECTED 0x040202 // CAN2 断连

#define IMU_NOT_FOUND 0x100201 // 未检测到 IMU
#define IMU_DISCONNECTED 0x100202 // IMU 断连
#define IMU_TIMEOUT 0x100303 // 超时无数据
#define IMU_DATA_INVALID 0x100304 // 数据异常（NaN/饱和）
#define IMU_CALIB_FAIL 0x100407 // 校准失败
#define IMU_TIME_SYNC_ERROR 0x100508 // 时间戳跳变
#define IMU_DRIVER_ERROR 0x100609 // SDK/协议解析失败

#define GPS_NOT_FOUND 0x110201 // 未检测到 GPS
#define GPS_DISCONNECTED 0x110202 // 断连
#define GPS_NO_SIGNAL 0x110304 // 无卫星 / SNR 太低
#define GPS_DATA_LOSS 0x110305 // 数据丢包
#define GPS_TIME_SYNC_ERROR 0x110508 // PPS/时间异常
#define GPS_DRIVER_ERROR 0x110609 // 协议解析失败（NMEA/UBlox）

// 左雷达
#define LIDAR1_NOT_FOUND 0x120201 // 未检测到雷达
#define LIDAR1_DISCONNECTED 0x120202 // 断连
#define LIDAR1_TIMEOUT 0x120303 // 无数据
#define LIDAR1_DATA_INVALID 0x120304 // 数据无效
#define LIDAR1_TIME_SYNC_ERROR 0x120508 // 时间戳异常
#define LIDAR1_DRIVER_ERROR 0x120609 // 协议解析失败
// 右雷达
#define LIDAR2_NOT_FOUND 0x130201 // 未检测到雷达
#define LIDAR2_DISCONNECTED 0x130202 // 断连
#define LIDAR2_TIMEOUT 0x130303 // 无数据
#define LIDAR2_DATA_INVALID 0x130304 // 数据无效
#define LIDAR2_TIME_SYNC_ERROR 0x130508 // 时间戳异常
#define LIDAR2_DRIVER_ERROR 0x130609 // 协议解析失败
// 左斜雷达
#define LIDAR3_NOT_FOUND 0x140201 // 未检测到雷达
#define LIDAR3_DISCONNECTED 0x140202 // 断连
#define LIDAR3_TIMEOUT 0x140303 // 无数据
#define LIDAR3_DATA_INVALID 0x140304 // 数据无效
#define LIDAR3_TIME_SYNC_ERROR 0x140508 // 时间戳异常
#define LIDAR3_DRIVER_ERROR 0x140609 // 协议解析失败
// 右斜雷达
#define LIDAR4_NOT_FOUND 0x150201 // 未检测到雷达
#define LIDAR4_DISCONNECTED 0x150202 // 断连
#define LIDAR4_TIMEOUT 0x150303 // 无数据
#define LIDAR4_DATA_INVALID 0x150304 // 数据无效
#define LIDAR4_TIME_SYNC_ERROR 0x150508 // 时间戳异常
#define LIDAR4_DRIVER_ERROR 0x150609 // 协议解析失败
// 后雷达
#define LIDAR5_NOT_FOUND 0x160201 // 未检测到雷达
#define LIDAR5_DISCONNECTED 0x160202 // 断连
#define LIDAR5_TIMEOUT 0x160303 // 无数据
#define LIDAR5_DATA_INVALID 0x160304 // 数据无效
#define LIDAR5_TIME_SYNC_ERROR 0x160508 // 时间戳异常
#define LIDAR5_DRIVER_ERROR 0x160609 // 协议解析失败

#define CAM1_NOT_FOUND 0x170201 // 未检测到相机
#define CAM1_DISCONNECTED 0x170202 // 断连
#define CAM1_FRAME_DROP 0x170305 // 丢帧过多
#define CAM1_DATA_INVALID 0x170304 // 图像损坏/格式错误
#define CAM1_TIME_SYNC_ERROR 0x170508 // 时间异常
#define CAM1_DRIVER_ERROR 0x170609 // SDK/驱动错误

#define CAM2_NOT_FOUND 0x180201 // 未检测到相机
#define CAM2_DISCONNECTED 0x180202 // 断连
#define CAM2_DEPTH_INVALID 0x180304 // 深度数据异常
#define CAM2_FRAME_DROP 0x180305 // 丢帧过多
#define CAM2_TIME_SYNC_ERROR 0x180508 // 时间异常
#define CAM2_DRIVER_ERROR 0x180609 // SDK/驱动错误

#define TFT_NOT_FOUND 0x190201 // 未检测到TFT
#define TFT_DISCONNECTED 0x190202 // 断连
#define TFT_DEPTH_INVALID 0x190304 //
#define TFT_FRAME_DROP 0x190305 //
#define TFT_TIME_SYNC_ERROR 0x190508 // 时间异常
#define TFT_DRIVER_ERROR 0x190609 // SDK/驱动错误

inline uint32_t encodeErrorLevel(uint32_t error_code, ErrorEventLevel level) {
    return (error_code << 8) | (static_cast<uint32_t>(level) & 0xFF);
}