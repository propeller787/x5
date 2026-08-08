# GateCenterlineCritic 改动说明

## 一键开关

| 方式 | 操作 |
|------|------|
| **Launch 参数（推荐）** | `gate_centerline_enable:=false` 关闭 / `:=true` 开启 |
| **YAML 参数** | 两个 MPPI 配置里改 `GateCenterlineCritic.enabled: false` |

```bash
# 关闭窄门中心线引导
ros2 launch wheelchair_obstacle mppi_stack_no_sensors.launch.py gate_centerline_enable:=false

# 仅启动 controller 层时
ros2 launch wheelchair_obstacle mppi_wheelchair_controller.launch.py gate_centerline_enable:=false
```

当前方出现窄门/窄道入口时，给 MPPI 多一个“更偏向入口中心进去”的软约束，尽量减少直接斜着贴边冲进去的情况。

## 这次实际涉及到的文件

### 1. 核心代码

`perception/wheelchair_obstacle/src/gate_centerline_critic.cpp`

- `GateCenterlineCritic` 主体逻辑；
- 负责窄门识别、摇杆意图匹配、中心线/航向代价计算；
- 带轻量 debug 输出；
- 代码里用 `==== 窄门中心线引导 cost 试验版 BEGIN/END ====` 标了新增范围。

`perception/wheelchair_obstacle/include/wheelchair_obstacle/gate_centerline_critic.hpp`

- critic 的类声明和参数定义；
- 也有 BEGIN/END 标记，方便后续往车上搬。

`perception/wheelchair_obstacle/gate_centerline_critic.xml`

- pluginlib 导出文件；
- 让 Nav2 MPPI 能加载这个 critic。

`perception/wheelchair_obstacle/CMakeLists.txt`

- 增加 `find_package(nav2_mppi_controller REQUIRED)`；
- 增加 `gate_centerline_critic` 动态库编译；
- 增加安装和 pluginlib 导出。

### 2. MPPI 配置

`perception/wheelchair_obstacle/config/mppi_controller_server.yaml`

- 主 MPPI 配置；
- 在 critics 列表里启用了 `GateCenterlineCritic`；
- 配了当前这一版的窄门识别、分阶段权重、debug 话题等参数。

`perception/wheelchair_obstacle/config/mppi_controller_server_localmap.yaml`

- localmap / occupymap 版本 MPPI 配置；
- 同样启用了 `GateCenterlineCritic`；
- 本地回放和另一套 costmap 运行方式时也要保持一致。

### 3. replay / debug 相关

`perception/wheelchair_obstacle/config/mppi_stack_bag_replay_record_topics.txt`

- 把 `/mppi/gate_centerline_critic/debug` 加进录包清单；
- replay 后可以直接判断这个 critic 有没有真的触发。



## 黑红需同步

新 cost 搬到试车代码里，同步下面 6 个：

1. `perception/wheelchair_obstacle/src/gate_centerline_critic.cpp`
2. `perception/wheelchair_obstacle/include/wheelchair_obstacle/gate_centerline_critic.hpp`
3. `perception/wheelchair_obstacle/gate_centerline_critic.xml`
4. `perception/wheelchair_obstacle/CMakeLists.txt`
5. `perception/wheelchair_obstacle/config/mppi_controller_server.yaml`
6. `perception/wheelchair_obstacle/config/mppi_controller_server_localmap.yaml`


## 当前代价的组成

基础上看，还是这两项：

```text
gate_cost =
    center_weight  * 中心线偏差
  + heading_weight * 航向偏差
```

做了分阶段加权：

- 远离入口时，权重低一些，避免太早被吸过去；
- 接近入口时，权重升高；
- 真正切入入口时，权重最高；
- 进入侧边窄道后，再逐渐回落。


## 当前触发条件

### 环境条件

- 入口宽度在 `0.80 ~ 1.50 m`；
- 入口距离在 `0.35 ~ 1.60 m`；
- 入口中心带没有被障碍堵死。

## 当前关键参数解释

- `critic_weight`
  - 这个 critic 的总话语权。

- `center_weight`
  - 偏离中心线时的惩罚强度。

- `heading_weight`
  - 车头没顺着入口方向时的惩罚强度。

- `active_x_band_m`
  - 在入口附近多大一段范围内明显起作用。

- `stage_prepare_ratio / stage_turn_ratio / stage_exit_ratio`
  - 分别控制准备阶段、切入阶段、退出阶段长度。

- `stage_*_center_scale`
  - 各阶段中心线代价倍率。

- `stage_*_heading_scale`
  - 各阶段航向代价倍率。

- `stage_weight_gain`
  - 关键阶段的额外加权。

- `critical_mix_ratio`
  - 最差采样点代价混入总分的比例，更盯“临门一脚”。

## debug 话题

调试话题：

```text
/mppi/gate_centerline_critic/debug
```

当前 `Float32MultiArray.data` 含义：

```text
[0] active
[1] distance_m
[2] width_m
[3] center_y_m
[4] axis_yaw_rad
[5] intent_forward
[6] intent_lateral
```
简单看：
- `active=1`：这帧真的启用了这个 critic；
- `distance_m`：识别到的入口距离；
- `width_m`：识别到的入口宽度；
- `center_y_m`：入口中心相对车体左右偏移，左正右负。
