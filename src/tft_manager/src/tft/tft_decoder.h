#pragma once

#include "tft_helper.h"
#include <string.h>
#include <sys/time.h>

namespace tft_service {

class TftDecoder {
    void decode_hrv(const char* buff, TFT_PACK& pt_info);
    bool get_hrv_data(const char* src_data, char* dst_data, int len);
};
}