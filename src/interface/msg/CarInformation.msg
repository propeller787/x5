std_msgs/Header header
#车辆IGN状态 点火开关 0 关  1 开
int32 ignstatus

#主车车速 km/h
float64 speed

#横摆角速度 
float64 yawrate 
#横摆角度 [-180,180]  
float64 roolyaw

#档位
#0.Park/Neutral-P档/N档
#1.1st-1档
#2.2nd-2档
#3.3rd-3档
#4.4th-4档
#5.5th-5档
#6.6th-6档
#7.Reverse
#8 D
int32 gearstate

#方向盘转角 (-780到779.9度/每位
float64 wheelangle

#制动踏板位移量 0 - 200mm
int32 brakpedaldisp

#油门踏板位置 0 - 200mm
int32 acceleratorpedal

#车辆GPS数据 经度
float64 longitude

#车辆GPS数据 维度
float64 latitude

# 左转向灯  0 关闭  1 开启
int32 leftturnlightst

# 右转向灯  0 关闭  1 开启
int32 rightturnlightst

#车辆 宽 米
float64 vehiclewide

#车辆 长 米
float64 vehiclelong

#车辆 后轴中心点到车头距离
float64 vehiclerearaxle