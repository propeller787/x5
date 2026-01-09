/*
 * @copyright: Copyright (c) 2024 zhengqi.com, Inc. All Rights Reserved
 * @Author: likang(likang@baidu.com)
 * @Date: 2024-01-05 16:32:38
 * @FilePath: mem/src/sensor/gps_component/f9k/gps_helper.h
 * @brief:
 */
#pragma once

#include "math.h"
#include "string.h"
#include <arpa/inet.h>
#include <iomanip>
#include <iostream>
#include <memory>

namespace sensor::gps {

struct GPSSetting {
    int distance;
    int mode;
};

typedef struct _GYRO {
    float gyrox; // 惯性器件x 轴角速度，单位 rad/s
    float gyroy; // 惯性器件y 轴角速度，单位 rad/s
    float gyroz; // 惯性器件z 轴角速度，单位 rad/s
} Gx_Gyro_Stru;
typedef struct _GX_ACC {
    float accx; // 惯性器件x 轴加速度，单位m/s2
    float accy; // 惯性器件y 轴加速度，单位m/s2
    float accz; // 惯性器件z 轴加速度，单位m/s2
} Gx_Acc_Stru;

typedef struct __LLA {
    float latitude; // 纬度
    float longitude; // 经度
    float height_over_sea; // 基于海平面的高度,单位m
} LLA_Stru;

typedef struct SPEED_Cog {
    float speed; // 地面速率，单位为m/s，固定输出3 位小数
    float cog; // 地面航向，单位为度，从北向起顺时针计算
} Speed_Cog_Stru;

/* GPS消息交互结构体 */
typedef struct {
    uint64_t frame_id; //
    uint64_t timestamp; // 时间戳
    float travel_distance; // 距离上次行驶距离
    LLA_Stru lla; // LLA 是 Latitude Longitude Altitude的缩写,意思 经纬度高度
    Speed_Cog_Stru speed_cog;
    Gx_Gyro_Stru gx_gyro; // 惯性器件 角速度，单位 rad/s
    Gx_Acc_Stru gx_acc; // 惯性器件 加速度，单位m/s2
} GPSData;

/* IMU消息交互结构体 */
typedef struct {
    uint64_t timestamp; // 本次时间戳,单位是us
    Gx_Gyro_Stru gx_gyro; // 惯性器件 角速度，单位 rad/s
    Gx_Acc_Stru gx_acc; // 惯性器件 加速度，单位m/s2
} IMUInfo;

typedef struct {
    uint32_t itow; // GPS 时间信息 (ms)
    uint16_t year; // 年
    uint8_t month; // 月
    uint8_t day; // 日
    uint8_t hour; // 小时
    uint8_t min; // 分钟
    uint8_t sec; // 秒
    uint8_t valid; // 有效性标志
    uint32_t tAcc; // 时间精度估计 (ns)
    int32_t nano; // 纳秒
    uint8_t fixType; // GNSS fix 类型
    uint8_t flags; // Fix 状态标志
    uint8_t flags2; // 附加标志
    uint8_t numSV; // 使用的卫星数量
    int32_t lon; // 经度 (1e-7 deg)
    int32_t lat; // 纬度 (1e-7 deg)
    int32_t height; // 椭球以上高度 (mm)
    int32_t hMSL; // 高于平均海平面高度 (mm)
    uint32_t hAcc; // 水平精度估计 (mm)
    uint32_t vAcc; // 垂直精度估计 (mm)
    int32_t velN; // 北向速度 (mm/s)
    int32_t velE; // 东向速度 (mm/s)
    int32_t velD; // 垂直下降速度 (mm/s)
    int32_t gSpeed; // 地速 (2D) (mm/s)
    int32_t headMot; // 运动航向 (1e-5 deg)
    uint32_t sAcc;
    uint32_t headAcc; // 速度精度估计 (mm/s)
    uint16_t pDop; // 位置 DOP (0.01)
    uint16_t flags3; // 附加标志3
    uint32_t reserved1; // 保留字段
    int32_t headVeh; // 车辆航向 (1e-5 deg)
    int16_t magDec; // 磁偏角 (1e-2 deg)
    uint16_t magAcc; // 磁偏角精度 (1e-2 deg)
} GPSPVT;

using GPSOriginDataPtr = std::shared_ptr<GPSData>;
}
