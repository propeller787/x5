# Visualization 工具包

这个目录可以单独拎出来用，主要包含两类能力：

1. `RViz + rosbag` 回放
2. `handle / cmd_vel / nav/control / odom` 曲线图可视化

## 目录说明

- `replay_bag_with_rviz.sh`
  直接在当前终端回放 bag，并打开 RViz。
- `run_replay_in_clean_terminal.sh`
  在一个干净的新终端里启动回放，平时推荐用这个。
- `monitor_playback_clock.py`
  回放时在终端里显示当前播到 bag 第几秒。
- `wheelchair_obstacle_debug.rviz`
  RViz 配置文件。
- `yisbot_o1.urdf`
  本地 `robot_state_publisher` 用的车体模型。
- `batch_plot_motion_signals_from_bags.py`
  批量生成 `handle / cmd / nav / odom` 四联图。
- `launch_rqt_plot_signals.sh`
  启动 `rqt_plot`，同步看 7 条时序曲线。
- `launch_plotjuggler.sh`
  启动 `PlotJuggler`，更适合多曲线联动分析。

## 环境依赖

最少需要：

- ROS 2 Humble
- `rviz2`
- `ros2 bag`
- Python 包：`rclpy`、`rosbag2_py`、`matplotlib`、`numpy`

如果要同步看曲线，还需要：

- `rqt_plot`
- `PlotJuggler`

## 回放命令

推荐：

```bash
bash run_replay_in_clean_terminal.sh \
  /path/to/bag_dir \
  0.4 --start-offset 35 --domain-id 66
```

含义：

- `0.4`：0.4 倍速
- `--start-offset 35`：从 bag 第 35 秒开始
- `--domain-id 66`：把本次回放隔离到单独 ROS 域，避免混到别的在线节点

默认行为：

- 先打开 RViz
- 启动本地 `robot_state_publisher`
- 终端里显示当前 bag 秒数
- 默认不会自动播放
- 你点一下播放终端，再按一次 `SPACE` 才正式开始

如果想一打开就自动播：

```bash
bash run_replay_in_clean_terminal.sh \
  /path/to/bag_dir \
  0.4 --start-offset 35 --domain-id 66 --auto-start
```

## 曲线图命令

批量画一个目录下所有 bag：

```bash
python3 batch_plot_motion_signals_from_bags.py /path/to/root_dir --x-tick-sec 1
```

输出文件会直接保存在对应根目录下：

- `<bag_name>_motion_signals_plot.png`
- `<bag_name>_motion_signals_plot.pdf`

## 同步看实时曲线

RViz 本身不擅长看时序曲线。更适合的工具是：

- `rqt_plot`
- `PlotJuggler`

如果你偏好多曲线联动、拖拽缩放更顺手，推荐优先用 `PlotJuggler`。

这个目录里已经准备了 `rqt_plot` 启动脚本：

```bash
bash launch_rqt_plot_signals.sh --domain-id 66
```

它会同时打开这些曲线：

- `/handle/control/linear/x`
- `/cmd_vel/linear/x`
- `/cmd_vel/angular/z`
- `/nav/control/linear/x`
- `/nav/control/angular/z`
- `/odom/fused/twist/twist/linear/x`
- `/odom/fused/twist/twist/angular/z`

注意：

- 回放和 `rqt_plot` 的 `ROS_DOMAIN_ID` 要一致
- 如果回放用的是 `--domain-id 66`，这里也要用 `66`

## PlotJuggler

本机如果还没装，可以安装：

```bash
sudo apt install ros-humble-plotjuggler ros-humble-plotjuggler-ros
```

启动：

```bash
bash launch_plotjuggler.sh --domain-id 66
```

启动后在 PlotJuggler 里选择 `ROS 2` data source，然后订阅这些曲线：

- `/handle/control/linear/x`
- `/cmd_vel/linear/x`
- `/cmd_vel/angular/z`
- `/nav/control/linear/x`
- `/nav/control/angular/z`
- `/odom/fused/twist/twist/linear/x`
- `/odom/fused/twist/twist/angular/z`

## 可选 overlay

如果你想让回放环境额外 source 某些工作区，可以在启动前设置：

```bash
export REPLAY_EXTRA_SETUP_FILES="/path/to/ws1/install/setup.bash:/path/to/ws2/install/setup.bash"
```

脚本会按顺序尝试 source。
