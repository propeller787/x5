/*
 * @copyright: Copyright (c) 2024 zhengqi.com, Inc. All Rights Reserved
 * @Author: likang(likang@baidu.com)
 * @Date: 2024-08-27 16:32:38
 * @FilePath: mem/src/sensor/gps_component/hrv/hrv_helper.h
 * @brief:
 */
#pragma once

#include "math.h"
#include "string.h"
#include <arpa/inet.h>
#include <iomanip>
#include <iostream>
#include <memory>

namespace tft_service {

typedef struct {
    uint8_t head1 = 0xAA; // 数据头
    uint8_t head2 = 0x55; // 数据头
    uint8_t page;         // 站号(页码)
    uint8_t len; // 数据长度（功能码+地址+数据+校验的总字节长度）
    uint8_t cmd; // 命令（功能码，如F1写寄存器 F2读寄存器）
    uint16_t address; // 变量地址（2字节）
    uint16_t data; // 数据（2字节）
    uint16_t crc = 0xCCCC; // 预留字段/校验（2字节，CRC未开启时固定为0xCCCC）
} TFT_PACK;

using healthOriginDataPtr = std::shared_ptr<TFT_PACK>;
}
