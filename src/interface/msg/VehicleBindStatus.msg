# 设备绑定
# 车辆唯一标识
string sn
# 服务器本地时间戳（毫秒）
int64 timestamp
# 车辆销售型号
string model
# 用户id
string user_id
# 状态0未绑定 1绑定 2变更(隐私协议变更)
int16 status
# 用勾选的隐私协议 逗号分隔
string privacy_policy