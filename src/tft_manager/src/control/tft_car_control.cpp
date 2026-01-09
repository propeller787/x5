#include "tft_car_control.h"
#include "../tft/tft_command_queue.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

using namespace tft_service::control;
using namespace std::placeholders;
CarControl::CarControl() : Control("CarControl") {
    using namespace std::placeholders;
    car_info_subs_ =
        global_->node_->create_subscription<interface::msg::CarInfo>(carInfoTopic, 2, std::bind(&CarControl::handleCarInfo, this, std::placeholders::_1));
    backup_car_subs_ = global_->node_->create_subscription<interface::msg::BackupCar>(backupCarTopic, 2,
                                                                                      std::bind(&CarControl::backupCarCallback, this, std::placeholders::_1));
}

CarControl::~CarControl() {}

void CarControl::handleCarInfo(const interface::msg::CarInfo msg) {
    car_info_ = msg;
    if (msg.speedkmh > 0.0 && global_->getType() == TFT_TYPE::OBSTACLE_AVOIDANCE) {
        // 倒车影像状态下  车速大于0  切换回标准界面
        global_->setType(tft_service::TFT_TYPE::STANDARD);
    }
    global_->setElectricQuantity(msg.electricquantity);
    // 充电状态
    if (!msg.getcharging) {
        sendChargingPage();
        return;
    }
    global_->sendAddressData(BATTERY_LEVEL_FRAME_ID, msg.electricquantity);
    if (global_->getType() != TFT_TYPE::VIDEO) {
        if (car_info_.electricquantity > 0 && car_info_.electricquantity <= 20 && car_info_.speedkmh > 0.0 && low_battery_20_) {
            low_battery_20_ = false;
            std_msgs::msg::Int32 audio_msg;
            audio_msg.data = AUDIO_TYPE_POWER_STATE_REMINDER_8;
            audio_pub_->publish(audio_msg);
            // global_->setType(tft_service::TFT_TYPE::ALARM);
            //  发送预警声音
        } else if (car_info_.electricquantity > 0 && car_info_.electricquantity <= 20 && car_info_.speedkmh == 0.0 && low_battery_20_) {
            // 发送预警声音
            std_msgs::msg::Int32 audio_msg;
            audio_msg.data = AUDIO_TYPE_POWER_STATE_REMINDER_8;
            audio_pub_->publish(audio_msg);
            // 发送预警界面
            std::cout << "电量低于20% 停车状态" << std::endl;
            global_->sendPageId(LOW_BATTERY_20_PAGE_ID);
            return;
            // 发送预警声音
        }

        if (car_info_.electricquantity > 0 && car_info_.electricquantity <= 10 && car_info_.speedkmh > 0.0 && low_battery_10_) {
            low_battery_10_ = false;
            std_msgs::msg::Int32 audio_msg;
            audio_msg.data = AUDIO_TYPE_POWER_STATE_REMINDER_7;
            audio_pub_->publish(audio_msg);
            // global_->setType(tft_service::TFT_TYPE::ALARM);
            //  发送预警界面
            //  发送预警声音
        } else if (car_info_.electricquantity > 0 && car_info_.electricquantity <= 10 && car_info_.speedkmh == 0.0 && low_battery_10_) {
            // 发送预警声音
            std_msgs::msg::Int32 audio_msg;
            audio_msg.data = AUDIO_TYPE_POWER_STATE_REMINDER_7;
            audio_pub_->publish(audio_msg);
            std::cout << "电量低于10% 停车状态" << std::endl;
            global_->sendPageId(LOW_BATTERY_PAGE_ID);
            return;
        }
    }
    if (!msg.lockswitch && global_->getType() == TFT_TYPE::OBSTACLE_AVOIDANCE) {
        global_->setType(TFT_TYPE::STANDARD);
    }
    if (global_->getType() == TFT_TYPE::STANDARD) {
        global_->sendNetworkStatus();
        if (msg.lockswitch) {
            // 未抱闸
            if (msg.speedkmh == 0.0) {
                std::cout << "停车状态" << std::endl;
                global_->sendPageId(PARKING_PAGE_ID);
                // 发送停车波浪
                sendParkingWave();
                // 发送剩余里程
                global_->sendAddressData(PARKING_MILEAGE_FRAME_ID, msg.remainingmileage);
                // 日期
                global_->sendDate();
                // 时间
                global_->sendTime();
                // 档位
                global_->sendAddressData(GEAR_FRAME_ID, msg.gear);
                global_->sendAddressData(TEMPERATURE_FRAME_ID, 20);
            } else if (msg.speedkmh > 0.0) {
                std::cout << "行驶状态" << std::endl;
                global_->sendPageId(MANUAL_DRIVE_PAGE_ID);
                sendSpeedKmh();
                global_->sendAddressData(GEAR_FRAME_ID, msg.gear);
                // 发送速度 km/h 档位
                global_->sendPageHeader();
            }
        } else {
            // 抱闸中
            if (car_info_.getcharging) {
                global_->sendPageId(PARKING_PAGE_ID_33);
                // 发送停车波浪
                sendParkingWave();
                // 发送剩余里程
                global_->sendAddressData(PARKING_MILEAGE_FRAME_ID, msg.remainingmileage);
                // 日期
                global_->sendDate();
                // 时间
                global_->sendTime();
                global_->sendAddressData(TEMPERATURE_FRAME_ID, 20);
                // 未充电  发送剩余里程 日期 时间
            } else {
                // 充电  发送剩余电量 日期  时间
            }
        }
    }
}

void CarControl::sendParkingWave() {
    const float TOTAL_MILEAGE = 30.0f; // 总里程30KM
    float percentage = (car_info_.remainingmileage / TOTAL_MILEAGE) * 100.0f;
    int16_t level = 0;
    if (percentage <= 10.0f) {
        level = 1; // 0-10%
    } else if (percentage <= 25.0f) {
        level = 2; // 11-25%
    } else if (percentage <= 50.0f) {
        level = 3; // 26-50%
    } else if (percentage <= 90.0f) {
        level = 4; // 51-90%
    } else {
        level = 5; // 91-100%
    }
    if (level != parking_wave_level_) {
        parking_wave_level_ = level;
        global_->sendAddressData(PARKING_WAVE_FRAME_ID, level);
    }
}

// 辅助函数：将32位单精度浮点数转换为16进制字节数组
std::vector<char> CarControl::floatToHexBytes(float value) {
    std::vector<char> bytes(4);

    // 使用联合体来避免类型转换的未定义行为
    union {
        float f;
        uint32_t i;
    } converter;

    converter.f = value;

    // 转换为小端序字节数组
    bytes[0] = (converter.i >> 24) & 0xFF; // 最高字节
    bytes[1] = (converter.i >> 16) & 0xFF;
    bytes[2] = (converter.i >> 8) & 0xFF;
    bytes[3] = converter.i & 0xFF; // 最低字节

    return bytes;
}

void CarControl::sendSpeedKmh() {
    // 5AA5 07 82 5420 42C6 00
    float speed_with_one_decimal = round(car_info_.speedkmh * 10.0f) / 10.0f;
    auto speed_bytes = floatToHexBytes(speed_with_one_decimal);
    if (speed_bytes.size() != 4) {
        RCLCPP_ERROR(logger_, "Speed conversion to bytes failed, expected 4 bytes but got %zu", speed_bytes.size());
        return;
    }
    char buf[10];
    buf[0] = 0x5A;
    buf[1] = 0xA5;
    buf[2] = 0x07;
    buf[3] = 0x82;
    buf[4] = 0x50;
    buf[5] = 0x20;
    buf[6] = speed_bytes[0];
    buf[7] = speed_bytes[1];
    buf[8] = speed_bytes[2];
    buf[9] = speed_bytes[3];
    // 使用命令队列发送原始数据
    TftCommand cmd(TftCommand::Type::RAW_DATA);
    cmd.data.assign(buf, buf + 10);
    global_->getCommandQueue()->enqueue(std::move(cmd));
}

void CarControl::backupCarCallback(const interface::msg::BackupCar::SharedPtr msg) {
    if (msg->is_backup_car) {
        // std::cout << "Backup car operation requested." << std::endl;
        RCLCPP_WARN(logger_, "Backup car operation requested.");
        global_->setType(tft_service::TFT_TYPE::VIDEO);
        sendOpenCamera();
    } else {
        // sendCloseCamera();
        global_->setType(tft_service::TFT_TYPE::STANDARD);
        RCLCPP_WARN(logger_, "Backup car operation cancelled.");
    }
}

void CarControl::sendOpenCamera() {
    global_->sendPageId(REVERSE_IMAGE_PAGE_ID);
    // 打开1号摄像头
    // 5A A5 0D 82 0120 80 08 08 02 51 80 00F0 00C8
    char buf3[16];
    buf3[0] = 0x5A;
    buf3[1] = 0xA5;
    buf3[2] = 0x0D;
    buf3[3] = 0x82;
    buf3[4] = 0x01;
    buf3[5] = 0x20;  // 地址
    buf3[6] = 0x80;  // 0 号设备开启
    buf3[7] = 0x48;  // 双字地址
    buf3[8] = 0x00;  //
    buf3[9] = 0x02;  // 固定
    buf3[10] = 0x51; // 固定
    buf3[11] = 0x80; // 左右镜像
    buf3[12] = 0x01; // 摄像头0 分辨率
    buf3[13] = 0xE0;
    buf3[14] = 0x01;
    buf3[15] = 0x68;
    // 使用命令队列发送原始数据
    TftCommand cmd(TftCommand::Type::RAW_DATA);
    cmd.data.assign(buf3, buf3 + 16);
    global_->getCommandQueue()->enqueue(std::move(cmd));
    // 开启总线 5A A5 07 82 01 00 A5 04 50 00
    char buf2[10];
    buf2[0] = 0x5A;
    buf2[1] = 0xA5;
    buf2[2] = 0x07;
    buf2[3] = 0x82;
    buf2[4] = 0x01;
    buf2[5] = 0x00;
    buf2[6] = 0xA5;
    buf2[7] = 0x04;
    buf2[8] = 0x50;
    buf2[9] = 0x00;
    // 使用命令队列发送原始数据
    TftCommand cmd2(TftCommand::Type::RAW_DATA);
    cmd2.data.assign(buf2, buf2 + 10);
    global_->getCommandQueue()->enqueue(std::move(cmd2));
}

void CarControl::sendCloseCamera() {
    char buf3[16];
    buf3[0] = 0x5A;
    buf3[1] = 0xA5;
    buf3[2] = 0x0D;
    buf3[3] = 0x82;
    buf3[4] = 0x01;
    buf3[5] = 0x20;  // 地址
    buf3[6] = 0x80;  // 1 号设备开启
    buf3[7] = 0x48;  // 双字地址
    buf3[8] = 0x00;  //
    buf3[9] = 0x02;  // 固定
    buf3[10] = 0x51; // 固定
    buf3[11] = 0x80; // 左右镜像
    buf3[12] = 0x01; // 摄像头0 分辨率
    buf3[13] = 0xE0;
    buf3[14] = 0x01;
    buf3[15] = 0x68;
    // 使用命令队列发送原始数据
    TftCommand cmd(TftCommand::Type::RAW_DATA);
    cmd.data.assign(buf3, buf3 + 16);
    global_->getCommandQueue()->enqueue(std::move(cmd));
}

void CarControl::sendChargingPage() {
    auto electricity = car_info_.electricquantity;
    int16_t index = 2; // 充电页面ID
    if (electricity <= 10.0f) {
        index = 2; // 0-10%
    } else if (electricity <= 25.0f) {
        index = 3; // 11-25%
    } else if (electricity <= 50.0f) {
        index = 4; // 26-50%
    } else if (electricity <= 90.0f) {
        index = 5; // 51-90%
    } else {
        index = 6; // 91-100%
    }
    if (index != charging_page_level_) {
        charging_page_level_ = index;
        global_->sendPageId(index);
    }
    global_->sendAddressData(BATTERY_LEVEL_FRAME_ID, car_info_.electricquantity);
    // 日期
    global_->sendDate();
    // 时间
    global_->sendTime();
    global_->sendNetworkStatus();
}