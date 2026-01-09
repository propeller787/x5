# Sensor 模块进程和线程分析报告

## 进程数量

**1 个进程**

- `main.cpp` 中使用 `rclcpp::executors::SingleThreadedExecutor`，所有节点运行在同一个进程中

## 线程数量

**总共 6 个线程**（1 个主线程 + 5 个工作线程）

### 主线程（1 个）

- **主执行线程**：`main.cpp` 中的 `SingleThreadedExecutor::spin()` 线程
  - 运行所有 ROS 节点的回调函数（定时器、订阅、服务等）
  - 文件：`src/sensor/main.cpp:57`

### 工作线程（5 个）

#### 1. ChassisComponent 相关线程（2 个）

**线程 1：ChassisTransreceiverThread**

- 类型：`muduo::net::EventLoopThread`
- 用途：CAN 总线收发器事件循环
- 创建位置：`src/sensor/chassis_component/chassis.cpp:11,130`
- 使用：用于处理 can0 和 can1 的 CAN 消息收发

**线程 2：ChassisAdapterThread**

- 类型：`muduo::net::EventLoopThread`
- 用途：底盘适配器事件循环
- 创建位置：`src/sensor/chassis_component/chassis.cpp:12,138`
- 使用：用于运行 ChassisCanAdapter 和 HandleCanAdapter 的 spin 循环

#### 2. GPS 组件相关线程（2 个）

**线程 3：GpsReceiverThread**

- 类型：`muduo::net::EventLoopThread`
- 用途：GPS 接收器事件循环
- 创建位置：`src/sensor/gps_component/gps.cpp:7,14`
- 使用：创建 EventLoop 并传递给 GPSModule

**线程 4：read*frame_loop* (GPSModule 内部)**

- 类型：`muduo::net::EventLoopThread`
- 用途：GPS 串口数据读取线程
- 创建位置：`src/sensor/gps_component/cs8/gps_module.cpp:244`
- 使用：执行 `thread_read_serial_data()` 函数，持续读取串口数据

#### 3. Camera 组件相关线程（1 个）

**线程 5：loop*thread* (H264UVC 内部)**

- 类型：`muduo::net::EventLoopThread`
- 用途：视频捕获线程
- 创建位置：`src/sensor/camera_component/h264_publisher.cpp:20`
- 使用：执行 `threadCapture()` 函数，从 V4L2 设备捕获 H264 视频流

### 不使用额外线程的组件

- **ImuComponent**：使用 ROS 定时器回调，在主线程中执行
- **SensorDataRecorder**：使用 ROS 订阅和定时器回调，在主线程中执行

## 线程分布汇总

| 组件               | 线程数量 | 线程类型               | 用途                      |
| ------------------ | -------- | ---------------------- | ------------------------- |
| 主线程             | 1        | SingleThreadedExecutor | ROS 回调执行              |
| ChassisComponent   | 2        | EventLoopThread        | CAN 收发、适配器          |
| GpsComponent       | 2        | EventLoopThread        | GPS 事件循环、串口读取    |
| CameraComponent    | 1        | EventLoopThread        | 视频捕获                  |
| ImuComponent       | 0        | -                      | 使用定时器（主线程）      |
| SensorDataRecorder | 0        | -                      | 使用订阅/定时器（主线程） |
| **总计**           | **6**    | -                      | -                         |

## 代码位置参考

- 主进程入口：`src/sensor/main.cpp:17-62`
- Chassis 线程：`src/sensor/chassis_component/chassis.cpp:11-12,129-144`
- GPS 线程：`src/sensor/gps_component/gps.cpp:7,14` 和 `src/sensor/gps_component/cs8/gps_module.cpp:244-247`
- Camera 线程：`src/sensor/camera_component/h264_publisher.cpp:20-24`
