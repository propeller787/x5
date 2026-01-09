#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cap_desc_parser.h"

//=======================================================================
//                                  多流能力描述解析器实现
//=======================================================================


int ParseMultiStreamConfig(unsigned char *pConfig,  struct MultiStreamCfg *Cfg_Desc)
{
    int i;
    unsigned char *pMultistreamCap;
    Cfg_Desc->NumStreams = pConfig[2];  // 从配置数据第三个字节获取流数量

    // 解析多流能力描述符
    Cfg_Desc->MS_Cap = malloc(Cfg_Desc->NumStreams * sizeof(struct MultiStreamCap)); // 为每个流分配内存
     pMultistreamCap = pConfig + pConfig[0];  // 定位到多流能力描述起始位置
     for(i=0; i<Cfg_Desc->NumStreams; i++)
     {
         // 从二进制数据拷贝12字节到结构体（跳过描述符头部2字节）
         memcpy((unsigned char *)&Cfg_Desc->MS_Cap[i], (unsigned char *)pMultistreamCap+2 ,12);
         pMultistreamCap += pMultistreamCap[0];  // 移动到下一个流描述符
     }
 
 }
 
 int ParseCapability(unsigned char *pData, int Length, struct CapabilityDescriptor *Cap_Desc)
 {
    
    int i, j;
    int ConfigLength;
    unsigned char *pConfigData, *pDemuxerData, *pFrameIntervalData, *pBitrateData;
    
    // 校验能力描述头格式
    if(pData[0] != MSC_HEADER_LENGTH || pData[1] != MSC_HEADER)
    {
        return -1;  // 头部长度或类型不匹配
    }
    
    // 解析头部信息
    Cap_Desc->NumConfigs = pData[4];  // 获取配置描述符数量

    // 解析配置描述符
    Cap_Desc->Cfg_Desc = malloc(Cap_Desc->NumConfigs * sizeof(struct MultiStreamCfg)); // 分配配置数组内存
    pConfigData = pData + pData[0];  // 定位到第一个配置描述符
    ConfigLength = pConfigData[0];   // 获取配置描述符长度
    for(i=0; i<Cap_Desc->NumConfigs; i++)
    {
        ParseMultiStreamConfig(pConfigData,  &Cap_Desc->Cfg_Desc[i]);  // 解析每个多流配置
        pConfigData += (ConfigLength + Cap_Desc->Cfg_Desc[i].NumStreams * pConfigData[ConfigLength]); // 移动到下一个配置
    }

    // 计算解复用器描述符数量
    pDemuxerData = pConfigData;
    i = 0;
    while(pDemuxerData[1]==MSC_DEMUXER)  // 遍历所有解复用器描述符
    {
        pDemuxerData += pDemuxerData[0];  // 按描述符长度跳转
        i++;
    }
    Cap_Desc->NumDemuxers = i;    

    // 解析解复用器描述符
    pDemuxerData = pConfigData;
    Cap_Desc->demuxer_Desc = malloc(Cap_Desc->NumDemuxers * sizeof(struct MultiStreamDemuxer));
    for(i = 0; i < Cap_Desc->NumDemuxers; i++)
    {
        // 解析解复用器参数
        Cap_Desc->demuxer_Desc[i].MSCDemuxIndex = pDemuxerData[2];  // 解复用器索引
        Cap_Desc->demuxer_Desc[i].DemuxID= pDemuxerData[3];        // 解复用器ID
        Cap_Desc->demuxer_Desc[i].Width = (pDemuxerData[4]<<8) | pDemuxerData[5];  // 图像宽度
        Cap_Desc->demuxer_Desc[i].Height= (pDemuxerData[6]<<8) | pDemuxerData[7];   // 图像高度
        pDemuxerData += pDemuxerData[0];  // 移动到下一个解复用器
    }

    // 计算帧间隔描述符数量    
    pFrameIntervalData = pDemuxerData;
    i = 0;
    while(pFrameIntervalData[1] == MSC_FRAMEINTERVAL)  // 遍历所有帧间隔描述符
    {
        pFrameIntervalData += pFrameIntervalData[0];    
        i++;
    }
    Cap_Desc->NumFrameIntervals = i;

    // 解析帧间隔描述符   
    pFrameIntervalData = pDemuxerData;
    Cap_Desc->FrameInt_Desc = malloc(Cap_Desc->NumFrameIntervals * sizeof(struct MultiStreamFrameInterval));
    for(i = 0; i < Cap_Desc->NumFrameIntervals; i++)
    {
        unsigned int FrameInternal, fps;
        Cap_Desc->FrameInt_Desc[i].FPSIndex = pFrameIntervalData[2];  // 帧率索引
        Cap_Desc->FrameInt_Desc[i].FPSCount = pFrameIntervalData[3]; // 支持的帧率数量
        Cap_Desc->FrameInt_Desc[i].FPS = malloc(Cap_Desc->FrameInt_Desc[i].FPSCount * sizeof(unsigned char));
        for(j = 0; j < Cap_Desc->FrameInt_Desc[i].FPSCount; j++)
        {
            // 从4字节数据计算帧间隔（单位：100ns）
            FrameInternal = (pFrameIntervalData[4+j*4]<<24) | (pFrameIntervalData[4+j*4+1]<<16) + 
                 (pFrameIntervalData[4+j*4+2]<<8) | pFrameIntervalData[4+j*4+3];
            fps = 10000000/FrameInternal;  // 转换为帧率（帧/秒）
            Cap_Desc->FrameInt_Desc[i].FPS[j] = (unsigned char)fps;    
        }
        pFrameIntervalData += pFrameIntervalData[0];  // 移动到下一个帧间隔描述符
    }

    // 计算码率描述符数量
    pBitrateData = pFrameIntervalData;
    i = 0;
    while(pBitrateData[1]==MSC_BITRATE)  // 遍历所有码率描述符
    {
        pBitrateData += pBitrateData[0];
        i++;
    }
    Cap_Desc->NumBitrate = i;
    
    // 解析码率描述符
    pBitrateData = pFrameIntervalData;
    Cap_Desc->Bitrate_Desc = malloc(Cap_Desc->NumBitrate*sizeof(struct MultiStreamBitrate));
    for(i = 0; i<Cap_Desc->NumBitrate; i++)
    {
        // 拷贝码率模式参数（2字节：索引+模式）
        memcpy(&Cap_Desc->Bitrate_Desc[i], pBitrateData+2, 2);
        pBitrateData += pBitrateData[0];  // 移动到下一个码率描述符
    }
    if((pBitrateData-pData)>Length)  // 校验数据长度
    {
        printf("%s:数据长度错误 \n",__FUNCTION__);
        return -1;
    }       
    free(pData);  // 释放原始数据缓冲区
    return 0;
}

//===================能力描述解析器结束====================================
