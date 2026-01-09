/*
 * @copyright: Copyright (c) 2024 zhengqi.com, Inc. All Rights Reserved
 * @Author: likang(likang@baidu.com)
 * @Date: 2024-01-05 16:32:38
 * @FilePath: mem/src/sensor/gps_component/f9k/gps_helper.h
 * @brief:
 */
#pragma once
#include "gps_helper.h"
#include <eigen3/Eigen/Dense>
#include <string.h>
#include <sys/time.h>
#include <vector>
namespace sensor::gps {

class GPSDecoder {
public:
    void decode_pvt(const char* buff, GPSData& gps_info);
    void decode_rmc(const char* buff, GPSData& gps_info);
    void decode_rtk(const char* buff, GPSData& gps_info);
    void decode_imu(const char* buff, IMUInfo& imu_msg, GPSData& gps_cur);
    bool Get_RMC_Data(const char* src_data, char* dst_data, int len);
    bool Get_PVT_Data(const char* src_data, char* dst_data, int len);
    bool Get_IMU_Data(const char* src_data, char* dst_data, int len);
    long creatTimestampMs();

    std::vector<std::string> split_string(const std::string& str, char delimiter);

private:
    // ddmm.mmm转换为dd.mmmmm的
    inline void caculate(float* x)
    {
        int a = 0;
        double b = 0, c = 0, d = 0, e = 0;
        a = *x / 100; // 30
        b = (int)((*x / 100 - a) * 100); // 29
        c = ((*x / 100 - a) * 100 - b) * 60;
        d = b / 60;
        e = c / 3600;
        *x = a + d + e;
    }
};

}