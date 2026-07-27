// error_code.hpp
#ifndef ERROR_CODE_HPP
#define ERROR_CODE_HPP

#include <string>
#include <cstdint>

// 错误码模块定义（与文档对应）
enum class ErrorModule : uint8_t
{
    SYSTEM = 0x01,   // 系统
    IMU = 0x10,      // IMU
    GPS = 0x11,      // GPS
    LIDAR1 = 0x12,   // 左雷达
    LIDAR2 = 0x13,   // 右雷达
    LIDAR3 = 0x14,   // 左斜雷达
    LIDAR4 = 0x15,   // 右斜雷达
    LIDAR5 = 0x16,   // 后雷达
    CAN0 = 0x02,     // CAN0
    CAN1 = 0x03,     // CAN1
    CAN2 = 0x04,     // CAN2
    DRIVER = 0x20,   // 电机驱动器
    TOP = 0x21,      // TOP驱动器
    OBSTACLE = 0x50, // 避障功能
    RECHARGE = 0x51  // 回充功能
};

// 错误码子模块定义
enum class ErrorSubModule : uint8_t
{
    HARDWARE = 0x01,      // 硬件
    DATA = 0x02,          // 数据
    TIMING = 0x03,        // 时间同步
    CONFIG = 0x04,        // 配置
    COMMUNICATION = 0x05, // 通信
    CALIBRATION = 0x06,   // 标定
    PERFORMANCE = 0x07,   // 性能
    POWER = 0x08,         // 电源
    TEMPERATURE = 0x09,   // 温度
    OTHER = 0x0A          // 其他
};

// 通用错误编号
enum class ErrorCode : uint16_t
{
    NO_ERROR = 0x0000,
    DEVICE_NOT_FOUND = 0x0001,
    DATA_INVALID = 0x0002,
    TIMEOUT = 0x0003,
    CONFIG_INVALID = 0x0004,
    COMMUNICATION_FAIL = 0x0005,
    CALIBRATION_FAIL = 0x0006,
    PERFORMANCE_LOW = 0x0007,
    POWER_ABNORMAL = 0x0008,
    TEMPERATURE_HIGH = 0x0009,
    UNKNOWN_ERROR = 0x000A
};

// 错误码数据结构
struct ErrorInfo
{
    uint32_t error_code;         // 完整错误码 0xMMSSEEEE
    std::string module_name;     // 模块名称
    std::string sub_module_name; // 子模块名称
    std::string description;     // 错误描述
    uint64_t timestamp;          // 时间戳（毫秒）
    bool is_recovered;           // 是否已恢复

    // 解析错误码
    ErrorModule getModule() const
    {
        return static_cast<ErrorModule>((error_code >> 24) & 0xFF);
    }

    ErrorSubModule getSubModule() const
    {
        return static_cast<ErrorSubModule>((error_code >> 16) & 0xFF);
    }

    uint16_t getErrorNumber() const
    {
        return error_code & 0xFFFF;
    }

    // 判断是否为严重错误
    bool isCritical() const
    {
        // 根据文档，硬件错误和部分功能错误为严重错误
        auto sub = getSubModule();
        auto module = getModule();

        // 硬件错误总是严重
        if (sub == ErrorSubModule::HARDWARE)
            return true;

        // 关键模块的通信错误
        if (sub == ErrorSubModule::COMMUNICATION)
        {
            return (module == ErrorModule::IMU ||
                    module == ErrorModule::LIDAR1 ||
                    module == ErrorModule::DRIVER);
        }

        // 回充功能相关的错误
        if (module == ErrorModule::RECHARGE)
        {
            uint16_t error_num = getErrorNumber();
            // 根据文档，回充功能错误码 0x0001-0x0003 为严重错误
            return (error_num >= 0x0001 && error_num <= 0x0003);
        }

        return false;
    }
};

// 错误码消息类型（用于ROS topic）
struct ErrorCodeMsg
{
    uint32_t error_code;
    uint64_t timestamp;
    uint8_t error_level; // 0: INFO, 1: WARNING, 2: ERROR, 3: CRITICAL
    std::string description;
};

#endif // ERROR_CODE_HPP