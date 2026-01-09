# Cloud Manager MQTT 上传数据分析

## 概述

本文档分析了 `cloud_manager` 模块通过 MQTT 上传到云端的所有数据类型、上传频率以及单次传输流量。

---

## 1. 定时上报数据（周期性）

### 1.1 实时车辆信息 (VehicleInfo)

**Topic**: `vehicle/up/{model}/{id}/{ver}/info`

**上传频率**: 每 **60 秒** 一次

**数据结构** (Protobuf):

- `BaseInfo`: vin, device_timestamp, model, version
- `speed_kmh`: 行驶速度 (float)
- `remaining_mileage`: 剩余里程 (int32)
- `gear`: 档位 (int32)
- `charging_status`: 充电状态 (int32)
- `parking_status`: 驻车状态 (int32)
- `assist_mode`: 辅助模式 (int32)
- `battery`: 电池电量 (int32)
- `left_brakests`: 左刹车状态 (int32)
- `right_brakests`: 右刹车状态 (int32)
- `total_mileage`: 总行驶里程 (float)
- `current_interval_mileage`: 当次行驶里程 (float)

**单次传输大小估算**:

- Protobuf 序列化数据: **约 80-120 字节**
- MQTT User Properties (gtime, gnonce, gsign): **约 100-150 字节**
- MQTT Topic 开销: **约 30-50 字节**
- **总计**: **约 210-320 字节/次**

**代码位置**: `cloud_manager_node.cpp:1252-1343` (`publish_realtime_msg()`)

---

### 1.2 域控制器数据 (DomainControllerData)

**Topic**: `vehicle/up/{model}/{id}/{ver}/dc`

**上传频率**: 每 **60 秒** 一次

**数据结构** (Protobuf):

- `BaseInfo`: vin, device_timestamp, model, version
- `cpu_usage`: CPU 利用率 (float)
- `mem_usage`: 内存利用率 (float)
- `network_status`: 网络状态 (uint32)
- `error_code`: 错误码 (int32)
- `robot_mode`: 当前模式字符串 (string, 可选)
- `vehicle_id`: 车辆 ID 字符串 (string, 可选)

**单次传输大小估算**:

- Protobuf 序列化数据: **约 60-110 字节** (取决于字符串字段)
- MQTT User Properties: **约 100-150 字节**
- MQTT Topic 开销: **约 30-50 字节**
- **总计**: **约 190-310 字节/次**

**代码位置**: `cloud_manager_node.cpp:1096-1225` (`publish_domain_controller()`)

---

### 1.3 GPS 数据 (GPSData)

**Topic**: `vehicle/up/{model}/{id}/{ver}/gps`

**上传频率**: 每 **60 秒** 一次

**数据结构** (Protobuf):

- `BaseInfo`: vin, device_timestamp, model, version
- `timestamp`: 采样时间戳 (double)
- `latitude`: 纬度 (double)
- `longitude`: 经度 (double)
- `altitude`: 海拔 (double)
- `velocity_n`: 北向速度分量 (float)
- `velocity_e`: 东向速度分量 (float)
- `velocity_u`: 垂直速度分量 (float)
- `fix_type`: 定位状态 (uint32)
- `num_satellites`: 卫星数量 (uint32)
- `hdop`: 水平精度因子 (float)

**单次传输大小估算**:

- Protobuf 序列化数据: **约 90-130 字节**
- MQTT User Properties: **约 100-150 字节**
- MQTT Topic 开销: **约 30-50 字节**
- **总计**: **约 220-330 字节/次**

**代码位置**: `cloud_manager_node.cpp:990-1081` (`publish_gps_tick()`)

---

## 2. 被动上报数据（事件触发）

### 2.1 车辆状态 (VehicleStatus)

**Topic**: `vehicle/up/{model}/{id}/{ver}/status`

**上传频率**: **状态变化时触发**（非固定频率）

**触发条件**:

- 首次开机必须上报一次
- `recharge_status` (回充状态) 变化
- `summon_status` (召唤状态) 变化
- `follow_me_status` (跟我走状态) 变化
- `current_mode_chinese` (模式名称) 变化
- `alarmstatus` (异常提醒) 变化

**数据结构** (Protobuf):

- `BaseInfo`: vin, device_timestamp, model, version
- `recharge_status`: 回充状态 (Int32Value, 可选)
- `summon_status`: 召唤状态 (Int32Value, 可选)
- `follow_me_status`: 跟我走状态 (Int32Value, 可选)
- `current_mode_chinese`: 中文模式名称 (StringValue, 可选)
- `alarmstatus`: 异常提醒 (Int32Value, 可选)

**单次传输大小估算**:

- Protobuf 序列化数据: **约 60-100 字节** (取决于哪些字段变化)
- MQTT User Properties: **约 100-150 字节**
- MQTT Topic 开销: **约 30-50 字节**
- **总计**: **约 190-300 字节/次**

**代码位置**: `cloud_manager_node.cpp:1404-1480` (`check_and_report_passive_status()`)

---

### 2.2 事件上报 (VehicleEventInfo)

**Topic**: `vehicle/event/{model}/{id}/{ver}/push`

**上传频率**: **事件触发时**（非固定频率，取决于事件发生频率）

**数据结构** (Protobuf):

- `sn`: 车辆唯一标识 (string)
- `device_timestamp`: 设备时间戳 (int64)
- `model`: 车辆型号 (string)
- `version`: 软件版本 (string)
- `config_version`: 配置文件版本 (int64)
- `event_id`: 事件唯一 ID (string)
- `event_type`: 事件类型 (int32)
- `duration_type`: 持续类型 (int32)
- `status`: 事件状态 (int32)
- `up_index`: 更新序号 (int64)
- `source`: 设备信息 (DeviceInfo, 可选)
- `safety_info`: 安全事件信息 (SafetyEventInfo, 可选)
- `alarm_info`: 报警信息 (AlarmInfo, 可选)
- `dtc_info`: 故障信息 (DTCInfo, 可选)
- `telemetry`: 遥测数据 (Telemetry, 可选)
- `log_file`: 日志文件列表 (repeated string, 可选)
- `video_file`: 视频文件列表 (repeated string, 可选)
- `extras`: 扩展信息 JSON (string, 可选)

**单次传输大小估算**:

- 基础字段: **约 100-200 字节**
- 包含可选字段（safety_info/alarm_info/dtc_info/telemetry）: **约 300-800 字节**
- 包含日志/视频文件列表: **约 500-2000 字节** (取决于文件数量)
- MQTT User Properties: **约 100-150 字节**
- MQTT Topic 开销: **约 40-60 字节**
- **总计**:
  - 最小: **约 240-410 字节/次**
  - 典型: **约 500-1000 字节/次**
  - 最大（含文件列表）: **约 2640-3210 字节/次**

**代码位置**: `cloud_manager_node.cpp:816-915` (`on_event_push_request()`)

---

### 2.3 事件配置回执 (EventConfigAck)

**Topic**: `vehicle/config/{model}/{ver}/ack`

**上传频率**: **收到事件配置时触发**（非固定频率）

**数据结构** (Protobuf):

- `config_id`: 配置唯一 ID (string)
- `timestamp`: 时间戳 (int64)
- `version`: 配置版本号 (string)
- `sn`: 车辆 SN (string)

**单次传输大小估算**:

- Protobuf 序列化数据: **约 50-80 字节**
- MQTT User Properties: **约 100-150 字节**
- MQTT Topic 开销: **约 30-50 字节**
- **总计**: **约 180-280 字节/次**

**代码位置**: `cloud_manager_node.cpp:742-749` (`send_event_config_ack()`)

---

## 3. 流量统计汇总

### 3.1 定时上报流量（每分钟）

| 数据类型         | 频率     | 单次大小     | 每分钟流量               |
| ---------------- | -------- | ------------ | ------------------------ |
| 实时车辆信息     | 1 次/60s | 210-320 字节 | **210-320 字节**         |
| 域控制器数据     | 1 次/60s | 190-310 字节 | **190-310 字节**         |
| GPS 数据         | 1 次/60s | 220-330 字节 | **220-330 字节**         |
| **定时上报小计** | -        | -            | **约 620-960 字节/分钟** |

### 3.2 被动上报流量（估算）

| 数据类型         | 触发条件 | 单次大小      | 估算频率         | 每分钟流量估算            |
| ---------------- | -------- | ------------- | ---------------- | ------------------------- |
| 车辆状态         | 状态变化 | 190-300 字节  | 0.1-1 次/分钟    | **19-300 字节/分钟**      |
| 事件上报         | 事件触发 | 240-3210 字节 | 0-5 次/分钟      | **0-16050 字节/分钟**     |
| 事件配置回执     | 收到配置 | 180-280 字节  | 0.01-0.1 次/分钟 | **2-28 字节/分钟**        |
| **被动上报小计** | -        | -             | -                | **约 21-16378 字节/分钟** |

### 3.3 总流量估算

**正常情况（无事件）**:

- 定时上报: 620-960 字节/分钟
- 被动上报: 21-300 字节/分钟
- **总计**: **约 641-1260 字节/分钟** ≈ **0.64-1.26 KB/分钟**

**高峰期（频繁事件）**:

- 定时上报: 620-960 字节/分钟
- 被动上报: 21-16378 字节/分钟
- **总计**: **约 641-17338 字节/分钟** ≈ **0.64-17.3 KB/分钟**

**每小时流量**:

- 正常情况: **约 38-76 KB/小时**
- 高峰期: **约 38-1040 KB/小时**

**每天流量**:

- 正常情况: **约 0.9-1.8 MB/天**
- 高峰期: **约 0.9-25 MB/天**

---

## 4. 关键代码位置

### 4.1 定时器初始化

- **文件**: `cloud_manager_node.cpp:461-477`
- **说明**: 所有定时上报任务的初始化

### 4.2 上报函数

- `publish_realtime_msg()`: 实时车辆信息上报 (line 1252)
- `publish_domain_controller()`: 域控制器数据上报 (line 1096)
- `publish_gps_tick()`: GPS 数据上报 (line 990)
- `publish_passive_msg()`: 被动消息上报模板函数 (line 1347)
- `check_and_report_passive_status()`: 车辆状态被动上报 (line 1404)

### 4.3 MQTT 发布

- **函数**: `mqtt_manager_->publish_with_properties()`
- **说明**: 所有 MQTT 消息都通过此函数发布，包含签名信息（gtime, gnonce, gsign）

---

## 5. 注意事项

1. **签名开销**: 每条消息都包含签名信息（gtime, gnonce, gsign），增加约 100-150 字节开销
2. **Protobuf 压缩**: Protobuf 使用变长编码，实际大小取决于数据值
3. **字符串字段**: 包含字符串的消息大小会随字符串长度变化
4. **事件上报**: 事件上报的大小变化范围很大，取决于是否包含日志/视频文件列表
5. **网络开销**: 上述估算仅包含 MQTT payload，实际网络传输还包括 MQTT 协议头、TCP/IP 头等

---

## 6. 优化建议

1. **压缩**: 可考虑对 Protobuf 数据进行 gzip 压缩，特别是事件上报数据
2. **批量上报**: 对于高频小数据，可考虑批量打包上报
3. **采样频率**: 根据实际需求调整定时上报频率（当前为 60 秒）
4. **事件过滤**: 对事件上报进行过滤，避免上报过多非关键事件

---

**文档生成时间**: 2025-01-27
**代码版本**: 基于 `cloud_manager_node.cpp` 分析
