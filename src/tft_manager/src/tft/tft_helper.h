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
    uint8_t head1 = 0x5A; // 数据头
    uint8_t head2 = 0xA5; // 数据头
    uint8_t len; // 数据长度
    uint8_t cmd; // 命令
    uint8_t address; // 地址
    uint8_t data; // 数据
} TFT_PACK;

using healthOriginDataPtr = std::shared_ptr<TFT_PACK>;
}
