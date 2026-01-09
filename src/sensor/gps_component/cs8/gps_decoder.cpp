// #pragma once
#include "gps_decoder.h"
#include <eigen3/Eigen/Dense>

namespace sensor::gps::cs8 {
/*
 *函数功能：解析pvt数据, 只需要速度和海拔
 */
void GPSDecoder::decode_pvt(const char* buff, GPSData& gps_info)
{
    gps_info.lla.height_over_sea = (*(int*)(buff)) * 0.001; // 单位是m
    gps_info.speed_cog.speed = (*(int*)(buff + 16)) * 0.001; // 对地速度，标量，单位是m/s
    // double north_velocity = (*(int*)(buff+4))*0.001;        //单位是m/s
    // double east_velocity  = (*(int*)(buff+8))*0.001;
    // double down_velocity  = (*(int*)(buff+12))*0.001;
    // velocity << north_velocity, east_velocity, down_velocity;
}

// 解码__RMC信息 $--RMC,time,status,Lat,N,Lon,E,spd,cog,date,mv,mvE,mode,navStates*cs
//$GNRMC,075625.00,A,4005.24825,N,11606.88641,E,1.311,35.05,230222,6.93,W,A,V*75
void GPSDecoder::decode_rmc(const char* buff, GPSData& gps_info)
{
    int ret = 0;
    int pos1 = -1;
    int pos2 = -1;
    std::string data_str(buff);
    std::cout << std::setprecision(15);

    if (data_str.size() < 1) {
        return;
    }

    // gps_data->status = (int)(data_str[17] - '0');
    pos1 = data_str.find(",", 19); // 维度坐标后的第一个“,”
    gps_info.lla.latitude = atof(data_str.substr(19, pos1 - 19).c_str());

    pos2 = pos1 + 3; // 经度坐标首字符位置
    pos1 = data_str.find(",", pos2); // 经度坐标后的第一个“,”
    gps_info.lla.longitude = atof(data_str.substr(pos2, pos1 - pos2).c_str());

    pos2 = pos1 + 3; // 速度首字符位置
    pos1 = data_str.find(",", pos2); // 速度后的第一个“,”
    // gps_data->speed = atof(data_str.substr(pos2, pos1-pos2).c_str());

    pos2 = pos1 + 1; // cog首字符位置
    pos1 = data_str.find(",", pos2); // cog后的第一个“,”
    gps_info.speed_cog.cog = atof(data_str.substr(pos2, pos1 - pos2).c_str());

    // std::cout << "RMC计算前-纬度：" << gps_data->latitude << "经度: " << gps_data->longitude << std::endl;
    caculate(&gps_info.lla.latitude);
    caculate(&gps_info.lla.longitude);
    // std::cout << "RMC计算后-纬度：" << gps_info.lla.latitude << "经度: " << gps_info.lla.longitude << std::endl;
    // gps_data->speed *= 1.852; //海里转化为公里, 转换后的单位 Km/h
}

/*
 *函数功能：从串口中读取的数据中解析出 XXRMC 字段
 */
bool GPSDecoder::Get_RMC_Data(const char* src_data, char* dst_data, int len)
{
    int pos = 0;
    int pos_end = 0;
    while (pos < (len - 9)) {
        if ((src_data[pos] == '$') && (src_data[pos + 3] == 'R') && (src_data[pos + 4] == 'M') && (src_data[pos + 5] == 'C')) {
            pos_end = (pos + 6);
            while (pos_end < (len - 2)) {
                if ((src_data[pos_end] == '*') && (src_data[pos_end + 2] != '\0')) {
                    memcpy(dst_data, src_data + pos, (pos_end - pos + 3));
                    printf("[GPS]Get_RMC_Data: dst_data = %s\n", dst_data);
                    return true;
                } else if (pos_end > (pos + 200)) {
                    printf("ERROR RMC Data, len = %d, pos_end = %d, pos = %d, data = %s\n", len, pos_end, pos, src_data);
                    break;
                }

                pos_end++;
            }
        }
        pos++;
    }
    return false;
}

/*
 *函数功能：从串口中读取的数据中解析出 PVT 字段
 */
bool GPSDecoder::Get_PVT_Data(const char* src_data, char* dst_data, int len)
{
    int pos = 0;
    printf("PVT数据: %s\n", src_data);
    // PVT数据长度是100个字节
    while (pos < (len - 98)) {
        if (((unsigned char)src_data[pos] == 0xB5) && ((unsigned char)src_data[pos + 1] == 0x62) && ((unsigned char)src_data[pos + 2] == 0x01) && ((unsigned char)src_data[pos + 3] == 0x07)) {
            printf("PVT--T L 符合, pos = %d, len = %d\n", pos, len);
            if ((pos < len - 99) && ((unsigned char)src_data[pos + 99] != '\0')) {
                memcpy(dst_data, &src_data[pos + 42], 4 * sizeof(char)); // 海平面高度,小端有符号32bit，单位mm
                memcpy(dst_data + 4, &src_data[pos + 54], 12 * sizeof(char)); // 三轴速度矢量，NED 东北地坐标，x轴指北，y轴指东，小端有符号32bit，单位mm/s
                memcpy(dst_data + 16, &src_data[pos + 66], 4 * sizeof(char)); // 对地标量速度，小端有符号32bit，单位mm/s
                return true;
            }
        }
        pos++;
    }
    return false;
}

std::vector<std::string> GPSDecoder::split_string(const std::string& str, char delimiter)
{
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        result.push_back(token);
    }

    return result;
}

}