# recharge

术语：
1. 基站：docking station
2. 雷达：lidar
3. 雷达方向： lidar heading


状态：
1. 充电中：     charging            CHARGING_DOING
2. 充电完成：   charging done       CHARGING_DONE
3. 充电失败：   charging failed     CHARGING_FAILED
4. 充电中止：   charging abort      CHARGING_ABORT
5. 充电暂停：   charging pause      CHARGING_PAUSE
6. 充电取消：   charging cancel     CHARGING_CANCEL
7.  充电开始：  charging start      CHARGING_START
8. 充电驾驶中： charging driving    CHARGING_DRIVING


colcon build --packages-select recharge


ros2 run recharge recharge_node


后续补充参数：
1. dx停止位。
2. 运动速度radio倍速。
3. 旋转速度radio倍速。
4. 角度修正。
5. 是否开启[旋转寻找]。
6. 是否开启[前移修正]功能。
7. [旋转寻找]等待秒数。
8. [旋转寻找]旋转速度。
9. [前移修正]前进速度。
10. [前移修正]前进秒数。
11. 开发[回充避障]功能。

代码后期改造，
1. 基站参数。
2. 车辆参数。
3. 环境参数。
4. 行走参数。

参数名	                            类型	默认值	说明
recharge_filter.min_index	            int	    80	    最小角度索引
recharge_filter.max_index	            int	    170	    最大角度索引
recharge_filter.min_intensities	        double	100.0	最小强度阈值
recharge_filter.max_intensities	        double	220.0	最大强度阈值
recharge_filter.min_distance	        double	0.20	最小有效距离
recharge_filter.max_distance	        double	3.5	    最大有效距离
recharge_filter.min_count	            int	    4	    最小连续点计数
recharge_filter.dock_length	            double	0.42	基站特征宽度
recharge_filter.baselink_back_index	    int	    225	    后向偏移索引
recharge_filter.dy_diff	                double	0.09	Y方向补偿 (= 0.1 - 0.01)
