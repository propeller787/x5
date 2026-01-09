# 单个模块的错误项
string module   # 模块名称，如 "LIDAR1"、"IMU"、"GPS"
uint32 code     # 原始错误码（十六进制对应的十进制值）
uint8 status_code    # 1=发生(YES), 2=解除(NO)