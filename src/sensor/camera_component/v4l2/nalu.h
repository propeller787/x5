#ifndef _NALU_H_
#define _NALU_H_

#include <stdbool.h>  // 用于布尔类型支持

/**
 * @brief 在H.264码流中查找下一个起始码
 * @param pBuf 当前搜索位置的指针
 * @param pBuf_end 缓冲区结束位置的指针
 * @return 找到的起始码地址(0x000001或0x0001)，未找到返回NULL
 * @note 起始码是H.264码流中NAL单元的分隔标记，格式为0x000001或0x0001
 */
unsigned char* FindNextH264StartCode(unsigned char *pBuf, unsigned char *pBuf_end);

/**
 * @brief 解析H.264的序列参数集(SPS)获取视频分辨率
 * @param buf SPS数据缓冲区指针
 * @param nLen SPS数据长度
 * @param Width 输出视频宽度
 * @param Height 输出视频高度
 * @return true解析成功 false解析失败
 * @note SPS包含视频编码的重要参数，本函数主要提取分辨率信息
 */
bool h264_decode_seq_parameter_set(unsigned char *buf, unsigned int nLen, int *Width, int *Height);

#if 0
// 保留的SPS结构定义（当前未使用）
/**
 * @brief H.264序列参数集(SPS)结构定义
 * @param profile_idc 编码配置标识符
 * @param constraint_setX_flag 约束集标志位(X=0-3)
 * @param reserved_zero_4bits 保留位（必须为0）
 * @param level_idc 编码等级标识符
 * @param seq_parameter_set_id SPS ID
 * @param log2_max_frame_num_minus4 计算最大帧号的参数
 */
typedef struct nalu_sps
{
    unsigned char profile_idc;              // 基准配置标识
    unsigned constraint_set0_flag:1;        // 约束集0标志
    unsigned constraint_set1_flag:1;        // 约束集1标志
    unsigned constraint_set2_flag:1;        // 约束集2标志
    unsigned constraint_set3_flag:1;        // 约束集3标志
    unsigned reserved_zero_4bits:4;          // 保留位
    unsigned char level_idc;                // 等级标识
    unsigned seq_parameter_set_id:1;        // SPS序列号
    unsigned log2_max_frame_num_minus4:5;   // 最大帧号对数参数
} NALU_SPS;
#endif

#endif // _NALU_H_
