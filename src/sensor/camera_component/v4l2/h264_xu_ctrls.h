#ifndef H264_XU_CTRLS_H
#define H264_XU_CTRLS_H

// 头文件包含
#include <linux/videodev2.h>      // V4L2视频设备相关定义
#include <linux/version.h>        // 内核版本信息
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 0)
#include <linux/uvcvideo.h>       // UVC视频驱动头文件（内核3.0+）
#endif

// 项目配置开关（0=常规项目 1=车载摄像头项目）
#define CARCAM_PROJECT				0

/*
 * 动态控制类型定义（从uvcvideo.h复制）
 * 用于描述UVC控制项的数据类型
 */
#define UVC_CTRL_DATA_TYPE_RAW		0   // 原始数据
#define UVC_CTRL_DATA_TYPE_SIGNED	1   // 有符号整型
#define UVC_CTRL_DATA_TYPE_UNSIGNED	2   // 无符号整型 
#define UVC_CTRL_DATA_TYPE_BOOLEAN	3   // 布尔类型
#define UVC_CTRL_DATA_TYPE_ENUM		4   // 枚举类型
#define UVC_CTRL_DATA_TYPE_BITMASK	5   // 位掩码类型

/* 自定义V4L2控制ID范围（基于扩展控制基址）+++++++++++++++++++++++++++++++++++++++ */
#define V4L2_CID_BASE_EXTCTR_RERVISION				0x0A0c4501  // 扩展控制基址
#define V4L2_CID_BASE_RERVISION						V4L2_CID_BASE_EXTCTR_RERVISION
#define V4L2_CID_ASIC_RW_RERVISION					V4L2_CID_BASE_RERVISION+1  // ASIC读写控制
#define V4L2_CID_FLASH_CTRL					    	V4L2_CID_BASE_RERVISION+2  // Flash控制
#define V4L2_CID_FRAME_INFO_RERVISION				V4L2_CID_BASE_RERVISION+3  // 帧信息
#define V4L2_CID_H264_CTRL_RERVISION				V4L2_CID_BASE_RERVISION+4  // H264编码控制
#define V4L2_CID_MJPG_CTRL_RERVISION				V4L2_CID_BASE_RERVISION+5  // MJPEG控制
#define V4L2_CID_OSD_CTRL_RERVISION					V4L2_CID_BASE_RERVISION+6  // 屏幕显示控制
#define V4L2_CID_MOTION_DETECTION_RERVISION			V4L2_CID_BASE_RERVISION+7  // 移动侦测
#define V4L2_CID_IMG_SETTING_RERVISION				V4L2_CID_BASE_RERVISION+8  // 图像设置
#define V4L2_CID_MULTI_STREAM_CTRL_RERVISION		V4L2_CID_BASE_RERVISION+9  // 多流控制
#define V4L2_CID_GPIO_CTRL_RERVISION				V4L2_CID_BASE_RERVISION+10 // GPIO控制
#define V4L2_CID_DYNAMIC_FPS_CTRL_RERVISION			V4L2_CID_BASE_RERVISION+11 // 动态FPS控制
#define V4L2_CID_LAST_EXTCTR_RERVISION				V4L2_CID_DYNAMIC_FPS_CTRL_RERVISION // 结束标记

/* XU控制单元GUID定义 ----------------------------------------------------------------- */
// 系统硬件控制GUID
#define UVC_GUID_RERVISION_SYS_HW_CTRL				{0x70,0x33,0xf0,0x28,0x11,0x63,0x2e,0x4a,0xba,0x2c,0x68,0x90,0xeb,0x33,0x40,0x16}
// 用户硬件控制GUID（当前生效）
#define UVC_GUID_RERVISION_USR_HW_CTRL				{0x94,0x73,0xDF,0xDD,0x3E,0x97,0x27,0x47,0xBE,0xD9,0x04,0xED,0x64,0x26,0xDC,0x67}
// 备选用户硬件控制GUID（已注释）
//#define UVC_GUID_RERVISION_USR_HW_CTRL			{0x3F,0xAE,0x12,0x28,0xD7,0xBC,0x11,0x4E,0xA3,0x57,0x6F,0x1E,0xDE,0xF7,0xD6,0x1D}

// XU控制单元ID
#define XU_RERVISION_SYS_ID							0x03  // 系统控制单元ID
#define XU_RERVISION_USR_ID							0x04  // 用户控制单元ID

/* ----------------------------- XU控制选择器定义 -------------------------------- */
// 系统控制选择器
#define XU_RERVISION_SYS_ASIC_RW	      		   0x01  // ASIC读写
#define XU_RERVISION_SYS_FLASH_CTRL				0x03  // Flash控制
#define XU_RERVISION_SYS_FRAME_INFO				0x06  // 帧信息
#define XU_RERVISION_SYS_H264_CTRL				0x07  // H264编码控制
#define XU_RERVISION_SYS_MJPG_CTRL				0x08  // MJPEG控制
#define XU_RERVISION_SYS_OSD_CTRL				0x09  // OSD控制
#define XU_RERVISION_SYS_MOTION_DETECTION		0x0A  // 移动侦测
#define XU_RERVISION_SYS_IMG_SETTING			0x0B  // 图像设置

// 用户控制选择器
#define XU_RERVISION_USR_FRAME_INFO				0x01  // 帧信息
#define XU_RERVISION_USR_H264_CTRL				0x02  // H264控制
#define XU_RERVISION_USR_MJPG_CTRL				0x03  // MJPEG控制
#define XU_RERVISION_USR_OSD_CTRL				0x04  // OSD控制
#define XU_RERVISION_USR_MOTION_DETECTION		0x05  // 移动侦测
#define XU_RERVISION_USR_IMG_SETTING			0x06  // 图像设置
#define XU_RERVISION_USR_MULTI_STREAM_CTRL		0x07  // 多流控制
#define XU_RERVISION_USR_GPIO_CTRL				0x08  // GPIO控制
#define XU_RERVISION_USR_DYNAMIC_FPS_CTRL		0x09  // 动态FPS控制

/* ----------------------------- UVC控制标志位定义 -------------------------------- */
// 从uvcvideo.h复制的控制标志位
#define UVC_CONTROL_SET_CUR	(1 << 0)  // 设置当前值
#define UVC_CONTROL_GET_CUR	(1 << 1)  // 获取当前值
#define UVC_CONTROL_GET_MIN	(1 << 2)  // 获取最小值
#define UVC_CONTROL_GET_MAX	(1 << 3)  // 获取最大值
#define UVC_CONTROL_GET_RES	(1 << 4)  // 获取分辨率
#define UVC_CONTROL_GET_DEF	(1 << 5)  // 获取默认值
#define UVC_CONTROL_RESTORE	(1 << 6)  // 休眠恢复时保留设置
#define UVC_CONTROL_AUTO_UPDATE	(1 << 7)  // 自动更新控制项

// 组合标志：获取所有范围参数
#define UVC_CONTROL_GET_RANGE   (UVC_CONTROL_GET_CUR | UVC_CONTROL_GET_MIN | \
                                 UVC_CONTROL_GET_MAX | UVC_CONTROL_GET_RES | \
                                 UVC_CONTROL_GET_DEF)

/* ----------------------------- 数据结构定义 -------------------------------- */
// UVC XU控制信息结构
struct uvc_xu_control_info {
	__u8 entity[16];  // 控制单元GUID
	__u8 index;       // 控制项索引
	__u8 selector;    // 控制选择器
	__u16 size;       // 数据大小
	__u32 flags;      // 控制标志位
};

// 内核版本3.0.0以下的映射结构定义
#if LINUX_VERSION_CODE <= KERNEL_VERSION (3, 0, 0)
struct uvc_xu_control_mapping {
	__u32 id;          // V4L2控制ID
	__u8 name[32];     // 控制项名称
	__u8 entity[16];   // 控制单元GUID
	__u8 selector;     // 控制选择器

	__u8 size;         // 数据大小
	__u8 offset;       // 数据偏移量
#if LINUX_VERSION_CODE == KERNEL_VERSION(3,0,0) || LINUX_VERSION_CODE == KERNEL_VERSION(3,0,35)
	__u32 v4l2_type;    // V4L2控制类型
	__u32 data_type;    // 数据类型
	struct uvc_menu_info __user *menu_info;  // 菜单项信息
	__u32 menu_count;   // 菜单项数量
	__u32 reserved[4];  // 保留字段
#else    
	enum v4l2_ctrl_type v4l2_type;  // V4L2控制类型
	__u32 data_type;                // 数据类型
#endif    
};
#endif

// UVC XU控制结构
struct uvc_xu_control {
	__u8 unit;     // 控制单元ID
	__u8 selector; // 控制选择器
	__u16 size;    // 数据大小
	__u8 *data;    // 数据指针
};

// H264格式描述结构
struct H264Format {
	unsigned short  wWidth;     // 视频宽度
	unsigned short  wHeight;    // 视频高度
	int   		fpsCnt;         // 帧率计数
	unsigned int	FrameSize;    // 帧尺寸
	unsigned int	*FrPay;       // 帧间隔/载荷指针（FrameInterval[0|1]Payloadsize[2|3]）
};

// 当前H264格式结构
struct Cur_H264Format {	
	int FmtId;                // 格式ID
	unsigned short wWidth;    // 视频宽度
	unsigned short wHeight;   // 视频高度
	int FrameRateId;          // 帧率ID
	unsigned char framerate;  // 帧率值
	unsigned int FrameSize;   // 帧尺寸
};

// 多流信息结构
struct Multistream_Info {
	unsigned char strm_type;  // 流类型
	unsigned int format;      // 格式信息
};

// 芯片型号枚举
typedef enum{
	CHIP_NONE = -1,     // 未识别
	CHIP_RER9420 = 0,   // RER9420芯片
	CHIP_RER9421,       // RER9421芯片
	CHIP_RER9422        // RER9422芯片
}CHIP_RER942X;

/* ----------------------------- IOCTL命令定义 -------------------------------- */
#if LINUX_VERSION_CODE > KERNEL_VERSION (3, 0, 0)
#define UVC_SET_CUR					0x01  // 设置当前值命令
#define UVC_GET_CUR					0x81  // 获取当前值命令
#define UVCIOC_CTRL_MAP		_IOWR('u', 0x20, struct uvc_xu_control_mapping)  // 映射控制
#define UVCIOC_CTRL_QUERY	_IOWR('u', 0x21, struct uvc_xu_control_query)   // 查询控制
#else  // 内核3.0以下版本
#define UVCIOC_CTRL_ADD		_IOW('U', 1, struct uvc_xu_control_info)    // 添加控制
#define UVCIOC_CTRL_MAP		_IOWR('U', 2, struct uvc_xu_control_mapping) // 映射控制
#define UVCIOC_CTRL_GET		_IOWR('U', 3, struct uvc_xu_control)         // 获取控制
#define UVCIOC_CTRL_SET		_IOW('U', 4, struct uvc_xu_control)          // 设置控制
#endif

/* ----------------------------- 全局变量声明 -------------------------------- */
extern unsigned int  chip_id;  // 存储检测到的芯片ID

/* ----------------------------- 函数声明 -------------------------------- */
// 基础控制函数
int XU_Init_Ctrl(int fd);                          // 初始化XU控制
int XU_Ctrl_ReadChipID(int fd);                    // 读取芯片ID

// H264格式相关函数
int H264_GetFormat(int fd);                        // 获取H264格式
int H264_CountFormat(unsigned char *Data, int len);// 计算格式数量
int H264_ParseFormat(unsigned char *Data, int len, struct H264Format *fmt); // 解析格式
int H264_GetFPS(unsigned int FrPay);               // 获取帧率

// XU通用控制接口
int XU_Set(int fd, struct uvc_xu_control xctrl);   // 设置XU控制
int XU_Get(int fd, struct uvc_xu_control *xctrl); // 获取XU控制
int XU_Set_Cur(int fd, __u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data); // 设置当前值
int XU_Get_Cur(int fd, __u8 xu_unit, __u8 xu_selector, __u16 xu_size, __u8 *xu_data); // 获取当前值

// H264编码控制
int XU_H264_InitFormat(int fd); // 初始化H264格式
int XU_H264_GetFormatLength(int fd, unsigned short *fwLen); // 获取格式数据长度
int XU_H264_GetFormatData(int fd, unsigned char *fwData, unsigned short fwLen); // 获取格式数据
int XU_H264_SetFormat(int fd, struct Cur_H264Format fmt); // 设置格式参数

// 量化参数控制
int XU_H264_Get_QP_Limit(int fd, int *QP_Min, int *QP_Max); // 获取QP范围
int XU_H264_Get_Mode(int fd, int *Mode);          // 获取编码模式
int XU_H264_Set_Mode(int fd, int Mode);           // 设置编码模式
int XU_H264_Get_QP(int fd, int *QP_Val);          // 获取当前QP值
int XU_H264_Set_QP(int fd, int QP_Val);           // 设置QP值

// 码率控制
int XU_H264_Get_BitRate(int fd, double *BitRate); // 获取码率
int XU_H264_Set_BitRate(int fd, double BitRate);  // 设置码率

// 帧控制
int XU_H264_Set_IFRAME(int fd);                   // 强制生成I帧

// SEI信息控制
int XU_H264_Get_SEI(int fd, unsigned char *SEI);  // 获取SEI信息
int XU_H264_Set_SEI(int fd, unsigned char SEI);    // 设置SEI信息

// GOP控制
int XU_H264_Get_GOP(int fd, unsigned int *GOP);   // 获取GOP值
int XU_H264_Set_GOP(int fd, unsigned int GOP);     // 设置GOP值

// ASIC读写
int XU_Asic_Read(int fd, unsigned int Addr, unsigned char *AsicData);  // 读取ASIC寄存器
int XU_Asic_Write(int fd, unsigned int Addr, unsigned char AsicData);   // 写入ASIC寄存器

/* 多流控制（仅在非车载项目启用）*/
#if(CARCAM_PROJECT == 0)
int XU_Multi_Get_status(int fd, struct Multistream_Info *status);      // 获取多流状态
int XU_Multi_Get_Info(int fd, struct Multistream_Info *Info);           // 获取多流信息
int XU_Multi_Set_Type(int fd, unsigned int format);                     // 设置多流类型

int XU_Multi_Set_Enable(int fd, unsigned char enable);                  // 启用多流
int XU_Multi_Get_Enable(int fd, unsigned char *enable);                 // 获取多流启用状态

// 多流码率控制
int XU_Multi_Set_BitRate(int fd, unsigned int StreamID, unsigned int BitRate); // 设置子码流码率
int XU_Multi_Get_BitRate(int fd, unsigned int StreamID, unsigned int *BitRate); // 获取子码流码率

// 多流QP控制
int XU_Multi_Set_QP(int fd, unsigned int StreamID, unsigned int QP_Val); // 设置子码流QP
int XU_Multi_Get_QP(int fd, unsigned int StreamID, unsigned int *QP_val); // 获取子码流QP

// 多流模式控制
int XU_Multi_Set_H264Mode(int fd, unsigned int StreamID, unsigned int Mode); // 设置子码流模式
int XU_Multi_Get_H264Mode(int fd, unsigned int StreamID, unsigned int *Mode); // 获取子码流模式

// 多流帧率控制
int XU_Multi_Set_SubStream_FrameRate(int fd, unsigned int sub_fps);     // 设置子码流帧率
int XU_Multi_Get_SubStream_FrameRate(int fd, unsigned int *sub_fps);    // 获取子码流帧率

// 多流GOP控制
int XU_Multi_Set_SubStream_GOP(int fd, unsigned int sub_gop);           // 设置子码流GOP
int XU_Multi_Get_SubStream_GOP(int fd, unsigned int *sub_gop);          // 获取子码流GOP
#endif

/* OSD屏幕显示控制 */
int XU_OSD_Timer_Ctrl(int fd, unsigned char enable);                    // 启用OSD计时器
int XU_OSD_Set_RTC(int fd, unsigned int year, unsigned char month, unsigned char day, 
                  unsigned char hour, unsigned char minute, unsigned char second); // 设置RTC时间
int XU_OSD_Get_RTC(int fd, unsigned int *year, unsigned char *month, unsigned char *day,
                  unsigned char *hour, unsigned char *minute, unsigned char *second); // 获取RTC时间

// OSD尺寸控制
int XU_OSD_Set_Size(int fd, unsigned char LineSize, unsigned char BlockSize); // 设置OSD尺寸
int XU_OSD_Get_Size(int fd, unsigned char *LineSize, unsigned char *BlockSize); // 获取OSD尺寸

// OSD颜色控制
int XU_OSD_Set_Color(int fd, unsigned char FontColor, unsigned char BorderColor); // 设置颜色
int XU_OSD_Get_Color(int fd, unsigned char *FontColor, unsigned char *BorderColor); // 获取颜色

// OSD启用控制
int XU_OSD_Set_Enable(int fd, unsigned char Enable_Line, unsigned char Enable_Block); // 启用OSD元素
int XU_OSD_Get_Enable(int fd, unsigned char *Enable_Line, unsigned char *Enable_Block); // 获取启用状态

// OSD自动缩放
int XU_OSD_Set_AutoScale(int fd, unsigned char Enable_Line, unsigned char Enable_Block); // 设置自动缩放
int XU_OSD_Get_AutoScale(int fd, unsigned char *Enable_Line, unsigned char *Enable_Block); // 获取自动缩放状态

// OSD位置控制
int XU_OSD_Set_Start_Position(int fd, unsigned char OSD_Type, unsigned int RowStart, 
                             unsigned int ColStart); // 设置起始位置
int XU_OSD_Get_Start_Position(int fd, unsigned int *LineRowStart, unsigned int *LineColStart,
                             unsigned int *BlockRowStart, unsigned int *BlockColStart); // 获取起始位置

/* 多流OSD控制（非车载项目）*/
#if(CARCAM_PROJECT == 0)
int XU_OSD_Set_Multi_Size(int fd, unsigned char Stream0, unsigned char Stream1, 
                         unsigned char Stream2); // 设置多流尺寸
int XU_OSD_Get_Multi_Size(int fd, unsigned char *Stream0, unsigned char *Stream1,
                         unsigned char *Stream2); // 获取多流尺寸

int XU_OSD_Set_MS_Start_Position(int fd, unsigned char StreamID, unsigned char RowStart,
                                unsigned char ColStart); // 设置多流起始位置
int XU_OSD_Get_MS_Start_Position(int fd, unsigned char *S0_Row, unsigned char *S0_Col,
                                unsigned char *S1_Row, unsigned char *S1_Col,
                                unsigned char *S2_Row, unsigned char *S2_Col); // 获取多流起始位置

// OSD字符串控制
int XU_OSD_Set_String(int fd, unsigned char group, char *String); // 设置OSD字符串
int XU_OSD_Get_String(int fd, unsigned char group, char *String); // 获取OSD字符串
#endif

/* 移动侦测控制 */
int XU_MD_Set_Mode(int fd, unsigned char Enable);          // 启用移动侦测
int XU_MD_Get_Mode(int fd, unsigned char *Enable);         // 获取移动侦测状态
int XU_MD_Set_Threshold(int fd, unsigned int MD_Threshold); // 设置侦测阈值
int XU_MD_Get_Threshold(int fd, unsigned int *MD_Threshold); // 获取侦测阈值
int XU_MD_Set_Mask(int fd, unsigned char *Mask);           // 设置侦测区域掩码
int XU_MD_Get_Mask(int fd, unsigned char *Mask);           // 获取侦测区域掩码
int XU_MD_Set_RESULT(int fd, unsigned char *Result);       // 设置侦测结果
int XU_MD_Get_RESULT(int fd, unsigned char *Result);       // 获取侦测结果

/* MJPEG控制 */
int XU_MJPG_Set_Bitrate(int fd, unsigned int MJPG_Bitrate); // 设置MJPEG码率
int XU_MJPG_Get_Bitrate(int fd, unsigned int *MJPG_Bitrate); // 获取MJPEG码率

/* 图像设置 */
int XU_IMG_Set_Mirror(int fd, unsigned char Mirror);       // 设置镜像模式
int XU_IMG_Get_Mirror(int fd, unsigned char *Mirror);       // 获取镜像模式
int XU_IMG_Set_Flip(int fd, unsigned char Flip);           // 设置翻转模式
int XU_IMG_Get_Flip(int fd, unsigned char *Flip);          // 获取翻转模式
int XU_IMG_Set_Color(int fd, unsigned char Color);         // 设置色彩模式
int XU_IMG_Get_Color(int fd, unsigned char *Color);         // 获取色彩模式

/* GPIO控制 */
int XU_GPIO_Ctrl_Set(int fd, unsigned char GPIO_En, unsigned char GPIO_Value); // 设置GPIO
int XU_GPIO_Ctrl_Get(int fd, unsigned char *GPIO_En, unsigned char *GPIO_OutputValue,
                    unsigned char *GPIO_InputValue); // 获取GPIO状态

/* 帧丢弃控制 */
int XU_Frame_Drop_En_Set(int fd, unsigned char Stream1_En, unsigned char Stream2_En); // 启用帧丢弃
int XU_Frame_Drop_En_Get(int fd, unsigned char *Stream1_En, unsigned char *Stream2_En); // 获取帧丢弃状态
int XU_Frame_Drop_Ctrl_Set(int fd, unsigned char Stream1_fps, unsigned char Stream2_fps); // 设置帧丢弃参数
int XU_Frame_Drop_Ctrl_Get(int fd, unsigned char *Stream1_fps, unsigned char *Stream2_fps); // 获取帧丢弃参数

/* Flash读写 */
int XU_SF_Read(int fd, unsigned int Addr, unsigned char* pData,unsigned int Length); // 读取Flash

/* 车载摄像头专用控制（仅在车载项目启用）*/
#if(CARCAM_PROJECT == 1)
int XU_OSD_Set_CarcamCtrl(int fd, unsigned char SpeedEn, unsigned char CoordinateEn,
                         unsigned char CoordinateCtrl); // 设置车载OSD控制
int XU_OSD_Get_CarcamCtrl(int fd, unsigned char *SpeedEn, unsigned char *CoordinateEn,
                         unsigned char *CoordinateCtrl); // 获取车载OSD控制
int XU_OSD_Set_Speed(int fd, unsigned int Speed);       // 设置车速显示
int XU_OSD_Get_Speed(int fd, unsigned int *Speed);      // 获取车速显示
int XU_OSD_Set_Coordinate1(int fd, unsigned char Direction, unsigned char *Vaule); // 设置坐标方式1
int XU_OSD_Set_Coordinate2(int fd, unsigned char Direction, unsigned char Vaule1,
                          unsigned long Vaule2, unsigned char Vaule3, unsigned long Vaule4); // 设置坐标方式2
int XU_OSD_Get_Coordinate1(int fd, unsigned char *Direction, unsigned char *Vaule); // 获取坐标方式1
int XU_OSD_Get_Coordinate2(int fd, unsigned char *Direction, unsigned char *Vaule1,
                          unsigned long *Vaule2, unsigned char *Vaule3, unsigned long *Vaule4); // 获取坐标方式2
#endif
int free264fmt();

#endif
