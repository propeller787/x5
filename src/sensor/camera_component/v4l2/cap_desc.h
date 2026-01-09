/* 视频能力描述相关宏定义 */
#define MAX_FORMAT  5   // 单个接口支持的最大视频格式数量（如H264/MJPEG等）
#define MAX_FRAME  10   // 单个视频格式支持的最大分辨率配置数量
#define MAX_FPS  10     // 单个分辨率配置支持的最大帧率配置数量

/* 原始二进制能力描述数据结构（从设备读取的未解析数据） */
struct CapabiltyBinaryData {
    unsigned char *pbuf;  // 指向原始二进制数据的指针
    int Lenght;           // 二进制数据长度（注意：原单词拼写错误，应为Length）
};

/* 帧类型描述结构体 - 描述特定分辨率下的帧率配置 */
struct FrameTypeDesc {
    unsigned short width;     // 帧宽度（单位：像素）
    unsigned short height;    // 帧高度（单位：像素）
    unsigned char NumFPS;     // 实际支持的帧率配置数量
    unsigned char FPS[MAX_FPS]; // 支持的帧率数组（单位：fps）
};

/* 接口描述结构体 - 描述UVC设备的视频接口能力 */
struct InterfaceDesc {
    unsigned char NumFormat;                  // 接口支持的视频格式数量
    unsigned int fmt[MAX_FORMAT];             // 视频格式数组（存储V4L2格式编码，如V4L2_PIX_FMT_H264）
    unsigned char NumFrame[MAX_FORMAT];        // 每个视频格式支持的分辨率配置数量
    struct FrameTypeDesc frame_info[MAX_FORMAT][MAX_FRAME]; // 分辨率配置二维数组[格式索引][分辨率索引]
};

/* 函数声明 */
// 获取设备的原始能力描述数据
int GetCapability(int fd, struct CapabiltyBinaryData *pCapData);
// 获取指定视频接口的描述信息
int GetInterface(int fd, struct InterfaceDesc *InterfaceDesc);
