std_msgs/Header header
#目标数据来源
# 1:camera
# 2:radar
# 3:lidar
# 101:radarFusion
# 102:cameraFusion
# 103:multiFusion
int32 source

#目标id
int64 id

#目标类型
# 0:人
# 10:红色定位点
# 20:关闭的电梯门
# 21:开启的电梯门
int32 type

#相对于车辆坐标系,单位m
float64 x
#相对于车辆坐标系,单位m
float64 y
#相对于车辆坐标系,单位m
float64 z 
#相对于车辆坐标系,单位度,车头方向为0度,顺时针360度
float64 heading
#速度x分量,单位m/s
float64 speedx
#速度y分量,单位m/s
float64 speedy
#速度,单位m/s
float64 speed
#目标长度,单位米
float64 length
#目标宽度,单位米
float64 width
#目标高度,单位米
float64 height
#目标扩展字段
ObjectExtend extend 
