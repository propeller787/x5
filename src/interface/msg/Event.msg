std_msgs/Header header
# STOP     停止
# RETARD   减速
# UPHILL   上坡
# DOWNHILL 下坡
# HUMP     颠簸
string type
# 危险等级 0:严重 1:告警 2：通知
int32 level

#警示文本
string toast
#警示语音
string voice