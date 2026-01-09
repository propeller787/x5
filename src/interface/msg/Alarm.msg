std_msgs/Header header
#告警uuid
string uuid
#告警编码
# 0:未知
# 1:超声波失联
# 2:激光雷达失联
# 3:相机失联
# 4:GPS失联
# 5:IMU失联
# 101:cpu温度过高
# 102:gpu温度过高
# 201:cpu使用率过高
# 202:gpu使用率过高
# 203:硬盘使用率过高
# 204:内存使用率过高
# 205:网卡使用率过高
# 301:时钟异常
# 401:通信异常
# 501:软件系统异常
# 601:相机出厂标定异常
# 602:雷达出厂标定异常
int32 alarm_code
#告警信息
string description
#告警等级
# 1:WARN
# 2:ERROR
# 3:FATAL
int32 alarm_level
