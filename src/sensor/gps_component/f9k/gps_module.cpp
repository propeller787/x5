#include "gps_module.h"

#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <unistd.h>
using namespace std;
namespace sensor::gps {

static muduo::net::EventLoopThread GpsReceiverThread;

int find_sequence(const char* data, int data_len, const char* sequence, int sequence_len)
{
    for (int i = 0; i <= data_len - sequence_len; i++) {
        if (memcmp(&data[i], sequence, sequence_len) == 0) {
            return i; // 返回序列的起始位置
        }
    }
    return -1; // 如果没有找到，返回-1
}

bool GPSModule::serial_data_analyse()
{
    bool ret = true;
    int len = 0;
    char once_data[REC_BUFF_LEN];
    char data_back[REC_BUFF_LEN];
    int len_back = 0;
    memset(once_data, 0, REC_BUFF_LEN * sizeof(char));
    len = _serial_port_ptr->read(once_data, REC_BUFF_LEN);
    bool status = false;

    if (len <= 0) {
        return false;
    } else {
        // cout << "读取的数据长度: " << len;

        char* gnrmc_pos = strstr(once_data, "$GNRMC");
        if (gnrmc_pos != nullptr) {
            char* end_pos = strchr(gnrmc_pos, '\r\n');
            if (end_pos != nullptr) {
                size_t len = end_pos - gnrmc_pos + 1; // 计算子字符串长度，包括 '\n'
                //_rmc_data[len + 1]; // 创建一个数组存储子字符串，加1是为了放'\0'
                // std::cout << " gnrmc_pos len : " << len << std::endl;
                strncpy(_rmc_data, gnrmc_pos, len); // 复制子字符串
                _rmc_data[len] = '\0'; // 手动添加字符串结束符
                // printf("Extracted RMC size : %d Data: %s\n", len, _rmc_data);
                //  return true;
                status = true;
            }
        }
        char* gngga_pos = strstr(once_data, "$GNGGA");
        if (gngga_pos != nullptr) {
            char* end_pos = strchr(gngga_pos, '\r\n');
            if (end_pos != nullptr) {
                size_t len = end_pos - gngga_pos + 1; // 计算子字符串长度，包括 '\n'
                //_rmc_data[len + 1]; // 创建一个数组存储子字符串，加1是为了放'\0'
                // std::cout << " gngga_pos len : " << len << std::endl;
                strncpy(_rtk_data, gngga_pos, len); // 复制子字符串
                _rtk_data[len] = '\0'; // 手动添加字符串结束符
                status = true;
            }
        }
        // unsigned char target_sequence[] = { 0xb5, 0x62, 0x01, 0x07 };
        int index = 0; // find_sequence(once_data, sizeof(once_data), target_sequence, sizeof(target_sequence));
        if (index == 0) {
            int pos = 0;
            if (((unsigned char)once_data[pos] == 0xB5) && ((unsigned char)once_data[pos + 1] == 0x62) && ((unsigned char)once_data[pos + 2] == 0x01) && ((unsigned char)once_data[pos + 3] == 0x07)) {
                memcpy(_pvt_data, &(once_data[6]), 92); // 复制子字符串
                memcpy(&gps_pvt_, _pvt_data, sizeof(GPSPVT));
                auto now = muduo::Timestamp::now();
                auto messageGps = sensor_msgs::msg::NavSatFix();
                auto messageImu = sensor_msgs::msg::Imu();
                messageGps.header.stamp.sec = now.secondsSinceEpoch();
                messageGps.header.stamp.nanosec = (now.microSecondsSinceEpoch() % 1000000) * 1000;
                messageGps.set__latitude(gps_pvt_.lat * 0.0000001);
                messageGps.set__longitude(gps_pvt_.lon * 0.0000001);
                messageGps.set__altitude(gps_pvt_.height * 0.001);
                // std::cout << std::fixed << std::setprecision(7) << " gps_pvt_.fixType " << (int)gps_pvt_.fixType << " gps_pvt_.flags " << (int)gps_pvt_.flags
                //           << " lat " << messageGps.latitude << " height " << messageGps.altitude << " valid " << gps_pvt_.valid
                //           << std::endl;
                int finxType = (int)gps_pvt_.fixType;
                switch (finxType) {
                case 0:
                    messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                case 2:
                    messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 3:
                    messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 4:
                    messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
                    break;
                case 5:
                    messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                default:
                    messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
                    break;
                }

                // 根据 flags 判断是否有 DGPS/RTK 标志
                if (((int)gps_pvt_.fixType) == 2 || ((int)gps_pvt_.fixType) == 3) {
                    if (((int)gps_pvt_.flags) == 131) { // 这里假设 flags 的第 0 位表示 DGPS
                        messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
                    }
                    if (gps_pvt_.flags & 0x02) { // 这里假设 flags 的第 1 位表示 RTK 修正
                        messageGps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
                    }
                }
                messageGps.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
                std::array<double, 9> arry;
                arry[0] = gps_pvt_.headMot * 0.00001;
                messageGps.set__position_covariance(arry);

                if (gps_pvt_.lat != 0 && gps_pvt_.lon != 0) {
                    cb_(messageGps);
                }
                // imucb_(messageImu);
                // std::cout << "Sending one GPS message." << std::endl;
                ++_index;
            }
        }
    }
    //

    return status;
}

void GPSModule::setGpsCallback(const onGpsCallback& cb)
{
    cb_ = cb;
}

void GPSModule::setImuCallback(const onImuCallback& cb)
{
    imucb_ = cb;
}

void GPSModule::thread_read_serial_data()
{
    while (true) {
        usleep(10000);
        // if (_sensor_global._gps_infos[get_device_id()].init_status != int(InitStatus::SUCCESS)) {
        //     /*TODO:Try to reinitialize GPS module*/
        //     continue;
        // }
        // char* arr = "123456789";
        // _serial_port_ptr->write(arr, 10);
        bool ret = serial_data_analyse();
        if (ret) {
            IMUInfo imu_msg = { 0 };
            GPSData gps_data_cur = { 0 };

            /*Decode message*/
            // _decoder.decode_pvt(_pvt_data, gps_data_cur);
            // _decoder.decode_imu(_imu_data, imu_msg, gps_data_cur);
            // _decoder.decode_rmc(_rmc_data, gps_data_cur);
            // _decoder.decode_rtk(_rtk_data, gps_data_cur);
            // auto now = muduo::Timestamp::now();
            // auto messageGps = sensor_msgs::msg::NavSatFix();
            // auto messageImu = sensor_msgs::msg::Imu();
            // messageGps.header.stamp.sec = now.secondsSinceEpoch();
            // messageGps.header.stamp.nanosec = (now.microSecondsSinceEpoch() % 1000000) * 1000;
            // messageGps.set__latitude(gps_data_cur.lla.latitude);
            // messageGps.set__longitude(gps_data_cur.lla.longitude);
            // messageGps.set__altitude(gps_data_cur.lla.height_over_sea);
            // std::array<double, 9> arry;
            // arry[0] = gps_pvt_.headMot * 0.00001;
            // messageGps.set__position_covariance(arry);
            // if (gps_data_cur.lla.latitude != 0 && gps_data_cur.lla.longitude != 0) {
            //     cb_(messageGps);
            // }
            // // imucb_(messageImu);
            // // std::cout << "Sending one GPS message." << std::endl;
            // ++_index;
        }
    }
}

// TODO: add error reconnect process
GPSModule::GPSModule(std::string device_address, muduo::net::EventLoop* loop)
{
    _buff = new char[REC_BUFF_LEN];
    memset(_buff, 0, REC_BUFF_LEN * sizeof(char));
    _total_len = 0;
    rtk_ptr_ = std::make_shared<QxwzSdkDemo>();
    rtk_ptr_->setRTKCallback(std::bind(&GPSModule::rtkWrite, this, _1, _2));
    OpenOptions gps_serial_port_option;
    gps_serial_port_option.auto_open = false;
    gps_serial_port_option.baud_rate = BR460800;
    gps_serial_port_option.data_bits = DATA_BITS_8;
    gps_serial_port_option.parity = PARITY_NONE;
    gps_serial_port_option.stop_bits = STOP_BITS_1;
    gps_serial_port_option.vmin = 0;
    gps_serial_port_option.vtime = 50;
    gps_serial_port_option.xany = false;
    gps_serial_port_option.xoff = false;
    gps_serial_port_option.xon = false;

    _index = 1;

    _serial_port_ptr = std::make_shared<SerialPort>(device_address, gps_serial_port_option);
    bool ret = _serial_port_ptr->start();
    if (!ret) {
        std::cout << "串口初始化失败" << std::endl;
    }
    auto pool = read_frame_loop_.startLoop();
    pool->runInLoop([=] {
        thread_read_serial_data();
    });

    // end of ctor
}

void GPSModule::rtkWrite(const char* buff, size_t len)
{
    int size = _serial_port_ptr->write((char*)buff, len);
    // std::cout << " rtk Write len " << size << std::endl;
}

GPSModule::~GPSModule()
{
    if (_buff) {
        delete[] _buff;
    }
}

} // namespace sensor::gps
