//=======================================================================
//                                  UVC设备能力描述解析器定义
//=======================================================================

// 能力描述符结构体 - 描述设备的多流配置能力
struct CapabilityDescriptor{
    unsigned char NumConfigs;                     // 多流配置数量
    struct MultiStreamCfg *Cfg_Desc;              // 多流配置描述数组指针
    unsigned char NumDemuxers;                    // 解复用器数量
    struct MultiStreamDemuxer *demuxer_Desc;      // 解复用器描述数组指针
    unsigned char NumFrameIntervals;              // 帧间隔配置数量
    struct MultiStreamFrameInterval *FrameInt_Desc; // 帧间隔描述数组指针
    unsigned char NumBitrate;                     // 码率配置数量
    struct MultiStreamBitrate *Bitrate_Desc;      // 码率描述数组指针
};

// 多流配置结构体 - 描述单个多流配置方案
struct MultiStreamCfg{
    unsigned char NumStreams;                     // 本配置包含的流数量
    struct MultiStreamCap *MS_Cap;               // 各流能力描述数组指针
};

// 单流能力描述结构体 - 描述单个视频流的配置参数
struct MultiStreamCap{
    unsigned char UVCInterfaceNum;               // UVC接口编号(0:主接口 1:次接口)
    unsigned char UVCFormatIndex;                // UVC格式描述符索引
    unsigned char UVCFrameIndex;                  // UVC帧描述符索引
    unsigned char DemuxerIndex;                  // 解复用器索引
    unsigned char FPSIndex;                      // 帧率配置索引
    unsigned char BRCIndex;                      // 码率控制配置索引
    unsigned char OSDIndex;                     // 屏幕显示(OSD)配置索引
    unsigned char MDIndex;                      // 移动检测(MD)配置索引
    unsigned char PTZIIndex;                    // 云台控制(PTZ)配置索引
    unsigned char FPSGroup;                     // 帧率分组标识
    unsigned char BRCGroup;                    // 码率控制分组标识
    unsigned char OSDGroup;                    // OSD分组标识
};

// 解复用器描述结构体 - 描述视频流解复用参数
struct MultiStreamDemuxer{
    unsigned char MSCDemuxIndex;                // 解复用器索引
    unsigned char DemuxID;                      // 解复用器ID
    unsigned short Width;                      // 视频流宽度
    unsigned short Height;                     // 视频流高度
};

// 帧间隔描述结构体 - 描述帧率配置信息
struct MultiStreamFrameInterval{
    unsigned char FPSIndex;                    // 帧率配置索引
    unsigned char FPSCount;                   // 支持的帧率数量
    unsigned char *FPS;                       // 帧率数组指针(单位：帧/秒)
};

// 码率描述结构体 - 描述码率控制参数
struct MultiStreamBitrate{
    unsigned char BRCIndex;                  // 码率配置索引
    unsigned char BRCMode;                   // 码率控制模式(如CBR/VBR)
};

// 能力描述区块类型枚举
enum{
	MSC_HEADER = 0,        // 头部信息区块
	MSC_CONFIG,            // 配置信息区块
	MSC_CAPABILITY,        // 能力描述区块
	MSC_DEMUXER,           // 解复用器描述区块
	MSC_FRAMEINTERVAL,     // 帧间隔描述区块
	MSC_BITRATE,           // 码率描述区块
	MSC_OSD                // 屏幕显示描述区块
};

#define MSC_HEADER_LENGTH 5  // 能力描述头部固定长度

// 能力描述解析函数声明
int ParseCapability(unsigned char *pCapability, int Length, struct CapabilityDescriptor *Cap_Desc);

//===================能力描述解析器定义结束====================================
