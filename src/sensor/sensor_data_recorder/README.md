# Sensor Data Recorder 操作文档

## 概述

Sensor Data Recorder 是一个统一的数据同步录制组件，用于同步录制 camera、imu、gps、can 等传感器数据到 rosbag2 文件，并支持后续通过 `ros2 bag play` 进行数据回放。

## 功能特性

- ✅ **多传感器同步录制**：同时录制 camera、imu、gps、can 数据
- ✅ **雷达数据独立录制**：支持独立录制 5 路雷达数据到单独的 bag 文件
- ✅ **时间戳同步**：使用消息原始时间戳，确保数据时间同步
- ✅ **自动文件管理**：支持按时间和大小自动分割 bag 文件
- ✅ **动态控制**：支持通过服务动态启动/停止录制（传感器数据和雷达数据分别控制）
- ✅ **参数配置**：支持通过 ROS2 参数动态修改配置
- ✅ **标准格式**：使用 SQLite3 格式，完全兼容 `ros2 bag play`

## 录制的话题

### 传感器数据话题

| 话题名称                     | 消息类型                                       | 说明                             |
| ---------------------------- | ---------------------------------------------- | -------------------------------- |
| `/sensor/backup/camera/h264` | `ffmpeg_image_transport_msgs/msg/FFMPEGPacket` | Camera H264 视频流               |
| `/sensor/imu/data`           | `sensor_msgs/msg/Imu`                          | IMU 数据（角速度、加速度、姿态） |
| `/sensor/gps/data`           | `sensor_msgs/msg/NavSatFix`                    | GPS 定位数据                     |
| `/can/frames`                | `can_msgs/msg/Frame`                           | CAN 总线数据                     |

### 雷达数据话题(后续可能会增加tf的)

| 话题名称        | 消息类型                    | 说明           |
| --------------- | --------------------------- | -------------- |
| `/scan/right`   | `sensor_msgs/msg/LaserScan` | 右侧雷达数据   |
| `/scan/left`    | `sensor_msgs/msg/LaserScan` | 左侧雷达数据   |
| `/scan/left_tilt` | `sensor_msgs/msg/LaserScan` | 左侧倾斜雷达数据 |
| `/scan/right_tilt` | `sensor_msgs/msg/LaserScan` | 右侧倾斜雷达数据 |
| `/scan/back`    | `sensor_msgs/msg/LaserScan` | 后侧雷达数据   |

**注意**：雷达数据使用独立的 bag 文件进行录制，与传感器数据分开存储。

## 配置参数

### 参数列表

| 参数名                        | 类型   | 默认值             | 说明                                  |
| ----------------------------- | ------ | ------------------ | ------------------------------------- |
| `record.directory`            | string | `/opt/freego/bags` | 录制文件保存目录                      |
| `record.bag_prefix`           | string | `sensor_data`      | 传感器数据 bag 文件前缀                |
| `record.max_duration_minutes` | int    | `30`               | 每个 bag 文件最大时长（分钟）         |
| `record.max_size_mb`          | int    | `1024`             | 每个 bag 文件最大大小（MB，默认 1GB） |
| `record.auto_start`           | bool   | `true`             | 启动时自动开始传感器数据录制          |
| `record.lidar.bag_prefix`    | string | `lidar_data`       | 雷达数据 bag 文件前缀                 |
| `record.lidar.auto_start`    | bool   | `true`             | 启动时自动开始雷达数据录制            |

### 参数设置方法

#### 方法 1：启动时通过命令行参数设置

```bash
ros2 run sensor sensor --ros-args \
  -p record.directory:=/home/user/bags \
  -p record.bag_prefix:=test_data \
  -p record.max_duration_minutes:=60 \
  -p record.max_size_mb:=2048 \
  -p record.auto_start:=true
```

#### 方法 2：运行时动态修改参数

```bash
# 修改录制目录
ros2 param set /sensor_data_recorder record.directory /home/user/new_bags

# 修改 bag 文件前缀
ros2 param set /sensor_data_recorder record.bag_prefix my_data

# 修改最大时长（分钟）
ros2 param set /sensor_data_recorder record.max_duration_minutes 60

# 修改最大大小（MB）
ros2 param set /sensor_data_recorder record.max_size_mb 2048

# 修改自动启动设置
ros2 param set /sensor_data_recorder record.auto_start false

# 修改雷达数据 bag 文件前缀
ros2 param set /sensor_data_recorder record.lidar.bag_prefix my_lidar_data

# 修改雷达数据自动启动设置
ros2 param set /sensor_data_recorder record.lidar.auto_start false
```

#### 方法 3：通过配置文件设置

创建参数文件 `sensor_recorder_params.yaml`：

```yaml
/sensor_data_recorder:
  ros__parameters:
    record:
      directory: "/opt/freego/bags"
      bag_prefix: "sensor_data"
      max_duration_minutes: 30
      max_size_mb: 1024
      auto_start: true
      lidar:
        bag_prefix: "lidar_data"
        auto_start: true
```

启动时加载配置文件：

```bash
ros2 run sensor sensor --ros-args --params-file sensor_recorder_params.yaml
```

## 服务接口

### 开始录制

**服务名称**：`/sensor/recorder/start`

**服务类型**：`std_srvs/srv/Empty`

**使用方法**：

```bash
ros2 service call /sensor/recorder/start std_srvs/srv/Empty
```

### 停止录制

**服务名称**：`/sensor/recorder/stop`

**服务类型**：`std_srvs/srv/Empty`

**使用方法**：

```bash
ros2 service call /sensor/recorder/stop std_srvs/srv/Empty
```

### 开始雷达数据录制

**服务名称**：`/sensor/recorder/lidar/start`

**服务类型**：`std_srvs/srv/Empty`

**使用方法**：

```bash
ros2 service call /sensor/recorder/lidar/start std_srvs/srv/Empty
```

### 停止雷达数据录制

**服务名称**：`/sensor/recorder/lidar/stop`

**服务类型**：`std_srvs/srv/Empty`

**使用方法**：

```bash
ros2 service call /sensor/recorder/lidar/stop std_srvs/srv/Empty
```

## 使用示例

### 示例 1：默认配置启动录制

```bash
# 启动 sensor 节点（默认会自动开始录制）
ros2 run sensor sensor

# 录制会自动开始，bag 文件保存在 /opt/freego/bags/ 目录下
# 文件命名格式：sensor_data_YYYYMMDD_HHMMSS
```

### 示例 2：手动控制录制

```bash
# 启动时禁用自动录制
ros2 run sensor sensor --ros-args -p record.auto_start:=false

# 手动开始录制
ros2 service call /sensor/recorder/start std_srvs/srv/Empty

# ... 进行数据采集 ...

# 手动停止录制
ros2 service call /sensor/recorder/stop std_srvs/srv/Empty
```

### 示例 3：自定义配置

```bash
# 设置自定义录制目录和文件前缀
ros2 run sensor sensor --ros-args \
  -p record.directory:=/home/user/my_bags \
  -p record.bag_prefix:=test_run_001 \
  -p record.max_duration_minutes:=60 \
  -p record.max_size_mb:=2048
```

### 示例 4：手动控制雷达数据录制

```bash
# 启动时禁用雷达数据自动录制
ros2 run sensor sensor --ros-args -p record.lidar.auto_start:=false

# 手动开始雷达数据录制
ros2 service call /sensor/recorder/lidar/start std_srvs/srv/Empty

# ... 进行数据采集 ...

# 手动停止雷达数据录制
ros2 service call /sensor/recorder/lidar/stop std_srvs/srv/Empty
```

### 示例 5：查看录制状态

```bash
# 查看当前参数配置
ros2 param list /sensor_data_recorder
ros2 param get /sensor_data_recorder record.directory
ros2 param get /sensor_data_recorder record.auto_start
ros2 param get /sensor_data_recorder record.lidar.auto_start

# 查看节点日志
ros2 topic echo /rosout | grep sensor_data_recorder
```

## 数据回放

### 查看 bag 文件信息

```bash
# 查看 bag 文件基本信息
ros2 bag info /opt/freego/bags/sensor_data_20250115_143022

# 输出示例：
# Files:             sensor_data_20250115_143022_0.db3
# Bag size:         1.2 GiB
# Storage id:        sqlite3
# Duration:          30.123s
# Start:             Jan 15 2025 14:30:22.123 (1736928622.123)
# End:               Jan 15 2025 14:30:52.246 (1736928652.246)
# Messages:          12543
# Topic information:
#   /sensor/backup/camera/h264: 4500 msgs    : ffmpeg_image_transport_msgs/msg/FFMPEGPacket
#   /sensor/imu/data: 301 msgs    : sensor_msgs/msg/Imu
#   /sensor/gps/data: 30 msgs    : sensor_msgs/msg/NavSatFix
#   /can/frames: 7712 msgs    : can_msgs/msg/Frame
```

#### 查看雷达数据 bag 文件信息

```bash
# 查看雷达数据 bag 文件基本信息
ros2 bag info /opt/freego/bags/lidar_data_20250115_143022

# 输出示例：
# Files:             lidar_data_20250115_143022_0.db3
# Bag size:         512 MiB
# Storage id:        sqlite3
# Duration:          30.123s
# Start:             Jan 15 2025 14:30:22.123 (1736928622.123)
# End:               Jan 15 2025 14:30:52.246 (1736928652.246)
# Messages:          15000
# Topic information:
#   /scan/right: 3000 msgs    : sensor_msgs/msg/LaserScan
#   /scan/left: 3000 msgs    : sensor_msgs/msg/LaserScan
#   /scan/left_tilt: 3000 msgs    : sensor_msgs/msg/LaserScan
#   /scan/right_tilt: 3000 msgs    : sensor_msgs/msg/LaserScan
#   /scan/back: 3000 msgs    : sensor_msgs/msg/LaserScan
```

### 回放数据

#### 基本回放

```bash
# 回放整个 bag 文件
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022
```

#### 回放选项

```bash
# 以原始速度回放
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 --rate 1.0

# 以2倍速度回放
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 --rate 2.0

# 以0.5倍速度回放（慢速回放）
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 --rate 0.5

# 只回放特定话题
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 \
  --topics /sensor/imu/data /sensor/gps/data

# 从指定时间开始回放（秒）
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 --start-offset 10

# 回放指定时长（秒）
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 --duration 60

# 循环回放
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 --loop

# 回放时发布到不同的话题前缀
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022 \
  --remap /sensor/imu/data:=/replay/imu/data
```

### 查看回放数据

```bash
# 查看 camera 数据
ros2 topic echo /sensor/backup/camera/h264

# 查看 IMU 数据
ros2 topic echo /sensor/imu/data

# 查看 GPS 数据
ros2 topic echo /sensor/gps/data

# 查看 CAN 数据
ros2 topic echo /can/frames

# 查看雷达数据
ros2 topic echo /scan/right
ros2 topic echo /scan/left
ros2 topic echo /scan/back

# 查看话题频率
ros2 topic hz /sensor/imu/data
ros2 topic hz /sensor/gps/data
ros2 topic hz /scan/right
```

#### 回放雷达数据

```bash
# 回放雷达数据 bag 文件
ros2 bag play /opt/freego/bags/lidar_data_20250115_143022

# 只回放特定雷达话题
ros2 bag play /opt/freego/bags/lidar_data_20250115_143022 \
  --topics /scan/right /scan/left

# 同时回放传感器数据和雷达数据（需要两个终端）
# 终端1：回放传感器数据
ros2 bag play /opt/freego/bags/sensor_data_20250115_143022

# 终端2：回放雷达数据
ros2 bag play /opt/freego/bags/lidar_data_20250115_143022
```

## 文件管理

### 文件命名规则

bag 文件命名格式：`{bag_prefix}_{YYYYMMDD}_{HHMMSS}`

**传感器数据文件示例**：

- `sensor_data_20250115_143022`
- `test_run_001_20250115_143022`

**雷达数据文件示例**：

- `lidar_data_20250115_143022`
- `my_lidar_data_20250115_143022`

### 自动分割

当满足以下任一条件时，会自动创建新的 bag 文件：

1. **时间限制**：当前 bag 文件录制时长达到 `max_duration_minutes`
2. **大小限制**：当前 bag 文件大小达到 `max_size_mb`

新文件会自动使用新的时间戳命名。

### 清理旧文件

```bash
# 查看录制目录下的所有 bag 文件
ls -lh /opt/freego/bags/

# 删除指定文件
rm -rf /opt/freego/bags/sensor_data_20250115_143022

# 删除7天前的文件
find /opt/freego/bags -type d -mtime +7 -exec rm -rf {} \;

# 按大小清理（删除大于10GB的文件）
find /opt/freego/bags -type d -size +10G -exec rm -rf {} \;
```

## 故障排查

### 问题 1：录制没有自动开始

**检查项**：

1. 确认 `record.auto_start` 参数是否为 `true`
2. 检查节点日志是否有错误信息
3. 确认录制目录是否有写权限

**解决方法**：

```bash
# 检查参数
ros2 param get /sensor_data_recorder record.auto_start

# 手动启动录制
ros2 service call /sensor/recorder/start std_srvs/srv/Empty
```

### 问题 2：bag 文件无法回放

**检查项**：

1. 确认 bag 文件格式是否正确（应为 SQLite3 格式）
2. 检查文件是否完整（没有损坏）

**解决方法**：

```bash
# 检查 bag 文件信息
ros2 bag info /path/to/bag_file

# 如果文件损坏，尝试重新录制
```

### 问题 3：录制目录空间不足

**检查项**：

1. 检查磁盘空间：`df -h /opt/freego/bags`
2. 检查当前 bag 文件大小

**解决方法**：

```bash
# 清理旧文件
find /opt/freego/bags -type d -mtime +7 -exec rm -rf {} \;

# 修改录制目录到有更多空间的路径
ros2 param set /sensor_data_recorder record.directory /path/to/larger/disk
```

### 问题 4：某些话题没有数据

**检查项**：

1. 确认话题是否正在发布：`ros2 topic list`
2. 检查话题频率：`ros2 topic hz /topic/name`
3. 查看节点日志

**解决方法**：

```bash
# 列出所有话题
ros2 topic list

# 检查传感器话题是否存在
ros2 topic info /sensor/imu/data
ros2 topic info /sensor/gps/data
ros2 topic info /can/frames
ros2 topic info /sensor/backup/camera/h264

# 检查雷达话题是否存在
ros2 topic info /scan/right
ros2 topic info /scan/left
ros2 topic info /scan/left_tilt
ros2 topic info /scan/right_tilt
ros2 topic info /scan/back

# 查看话题发布频率
ros2 topic hz /sensor/imu/data
ros2 topic hz /scan/right
```

### 问题 5：雷达数据没有录制

**检查项**：

1. 确认 `record.lidar.auto_start` 参数是否为 `true`
2. 检查雷达话题是否正在发布
3. 查看节点日志是否有雷达录制相关错误

**解决方法**：

```bash
# 检查参数
ros2 param get /sensor_data_recorder record.lidar.auto_start

# 手动启动雷达数据录制
ros2 service call /sensor/recorder/lidar/start std_srvs/srv/Empty

# 检查雷达话题
ros2 topic list | grep scan
ros2 topic hz /scan/right
```

## 性能优化建议

1. **磁盘 I/O 优化**：

   - 使用 SSD 存储 bag 文件
   - 避免在录制目录进行其他高 I/O 操作

2. **文件大小管理**：

   - 根据实际需求设置合理的 `max_duration_minutes` 和 `max_size_mb`
   - 定期清理旧文件，避免磁盘空间不足

3. **网络优化**（如果使用网络存储）：

   - 使用本地存储优先
   - 如果必须使用网络存储，确保网络带宽充足

4. **系统资源**：
   - 监控 CPU 和内存使用情况
   - 确保系统有足够的资源处理数据录制

## 注意事项

1. ⚠️ **磁盘空间**：确保录制目录有足够的磁盘空间，建议至少保留 10GB 以上空间（传感器数据和雷达数据会分别占用空间）
2. ⚠️ **时间同步**：确保系统时间准确，bag 文件使用系统时间戳
3. ⚠️ **权限问题**：确保对录制目录有写权限
4. ⚠️ **数据完整性**：停止录制时使用服务接口，避免直接终止进程导致数据损坏
5. ⚠️ **话题订阅**：确保所有需要录制的话题都在正常运行
6. ⚠️ **独立录制**：传感器数据和雷达数据使用独立的 bag 文件，需要分别控制录制和回放

## 相关命令速查

```bash
# 启动录制（默认自动开始）
ros2 run sensor sensor

# 开始传感器数据录制
ros2 service call /sensor/recorder/start std_srvs/srv/Empty

# 停止传感器数据录制
ros2 service call /sensor/recorder/stop std_srvs/srv/Empty

# 开始雷达数据录制
ros2 service call /sensor/recorder/lidar/start std_srvs/srv/Empty

# 停止雷达数据录制
ros2 service call /sensor/recorder/lidar/stop std_srvs/srv/Empty

# 查看参数
ros2 param list /sensor_data_recorder
ros2 param get /sensor_data_recorder record.directory

# 修改参数
ros2 param set /sensor_data_recorder record.directory /new/path

# 查看 bag 信息
ros2 bag info /path/to/bag_file

# 回放 bag
ros2 bag play /path/to/bag_file

# 查看话题列表
ros2 topic list

# 查看话题信息
ros2 topic info /topic/name

# 查看话题频率
ros2 topic hz /topic/name
```

## 版本信息

- **组件名称**：sensor_data_recorder
- **节点名称**：/sensor_data_recorder
- **支持格式**：rosbag2 (SQLite3)
- **ROS2 版本**：Humble/Foxy（根据实际版本调整）

---

**最后更新**：2025-01-15
