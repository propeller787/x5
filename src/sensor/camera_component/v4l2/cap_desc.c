#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include "debug.h"
#include "h264_xu_ctrls.h"
#include "cap_desc.h"

/* 获取设备能力描述数据 */
int GetCapability(int fd, struct CapabiltyBinaryData *pCapData)
{
    unsigned int SensorInitTableAddr, CapAddr, CapSize, Section11_addr;
    unsigned char *pBuffer;
    
    /* 步骤1：获取传感器初始化表地址 */
    // 分配4字节缓冲区用于读取传感器初始化表地址
    pBuffer = malloc(4 * sizeof(unsigned char));
    if(XU_SF_Read(fd, 0x017f, pBuffer,4)<0)  // 从设备0x017f寄存器读取地址
    {
        TestAp_Printf(TESTAP_DBG_ERR, "%s:read SF error\n",__FUNCTION__);
        return -1;
    }
    // 将4字节数据组合成32位地址（大端格式）
    SensorInitTableAddr = (pBuffer[0]<<24) | (pBuffer[1]<<16) | (pBuffer[2]<<8) | pBuffer[3];
    TestAp_Printf(TESTAP_DBG_BW, "%s: SensorInitTableAddr = 0x%x\n",__FUNCTION__, SensorInitTableAddr);
    free(pBuffer);  // 释放临时缓冲区

    /* 步骤2：获取能力描述区域地址和大小 */
    pBuffer = malloc(25 * sizeof(unsigned char));   // 分配25字节缓冲区
    if(XU_SF_Read(fd, SensorInitTableAddr, pBuffer,25)<0)  // 读取初始化表内容
    {
        TestAp_Printf(TESTAP_DBG_ERR, "%s:read SF error\n",__FUNCTION__);
        return -1;
    }
    // 解析能力描述区域地址（从缓冲区21-22字节获取偏移量）
    CapAddr = SensorInitTableAddr + ((pBuffer[21]<<8) | pBuffer[22]);
    // 解析Section11地址（从缓冲区23-24字节获取偏移量）
    Section11_addr  = SensorInitTableAddr + ((pBuffer[23]<<8) | pBuffer[24]);
    CapSize = Section11_addr - CapAddr;  // 计算能力描述数据区域大小
    TestAp_Printf(TESTAP_DBG_BW, "%s: CapAddr = 0x%x, CapSize = 0x%x\n",__FUNCTION__, CapAddr, CapSize);
    free(pBuffer);  // 释放临时缓冲区
    
    /* 步骤3：读取完整的能力描述数据 */
    pCapData->pbuf= malloc(CapSize * sizeof(unsigned char)); // 分配存储空间
    pCapData->Lenght= CapSize;  // 记录数据长度（注意：原单词拼写错误，应为Length）
    
    if(XU_SF_Read(fd, CapAddr, pCapData->pbuf,CapSize)<0)  // 读取能力描述数据
    {
        TestAp_Printf(TESTAP_DBG_ERR, "%s:read SF error\n",__FUNCTION__);
        return -1;
    }
    return 0;
}

/* 获取视频接口描述信息 */
int GetInterface(int fd,struct InterfaceDesc *Interface_Desc)
{
        int ret, i, j, k;
        struct v4l2_fmtdesc fmt;    // 视频格式描述结构体
        struct v4l2_frmsizeenum fsize; // 帧尺寸枚举结构体
        struct v4l2_frmivalenum fival; // 帧间隔（帧率）枚举结构体

        memset(Interface_Desc, 0 , sizeof(struct InterfaceDesc));  // 初始化接口描述结构体

        /* 枚举所有支持的视频格式 */
        memset(&fmt, 0, sizeof(fmt));
        fmt.index = 0;  // 从第一个格式开始枚举
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  // 指定视频捕获类型
        while ((ret = ioctl(fd, VIDIOC_ENUM_FMT, &fmt)) == 0) {  // 循环获取所有格式
            Interface_Desc->NumFormat ++;  // 增加格式计数
            Interface_Desc->fmt[fmt.index] = fmt.pixelformat; // 存储格式编码（如H264）
            fmt.index++;  // 准备获取下一个格式
        }

        /* 枚举每个视频格式支持的分辨率 */
        memset(&fsize, 0, sizeof(fsize));
        for(i = 0; i < Interface_Desc->NumFormat;i++)  // 遍历所有视频格式
        {
            fsize.index = 0;  // 从第一个分辨率开始枚举
            fsize.pixel_format = Interface_Desc->fmt[i];  // 设置当前视频格式
            while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fsize)) == 0) 
            {
                if (fsize.type == V4L2_FRMSIZE_TYPE_DISCRETE)  // 仅处理离散分辨率
                { 
                    Interface_Desc->NumFrame[i]++;  // 增加当前格式的分辨率计数
                    // 存储分辨率信息
                    Interface_Desc->frame_info[i][fsize.index].width = fsize.discrete.width;
                    Interface_Desc->frame_info[i][fsize.index].height= fsize.discrete.height;
                }
                fsize.index++;  // 准备获取下一个分辨率
            }
        }

        /* 枚举每个分辨率支持的帧率 */
        memset(&fival, 0, sizeof(fival));
        for(i = 0; i < Interface_Desc->NumFormat;i++)  // 遍历所有视频格式
        {
            for(j = 0; j < Interface_Desc->NumFrame[i];j++)  // 遍历当前格式的所有分辨率
            {
                fival.index = 0;  // 从第一个帧率开始枚举
                fival.pixel_format = Interface_Desc->fmt[i];  // 设置视频格式
                fival.width = Interface_Desc->frame_info[i][j].width;  // 设置分辨率宽度
                fival.height = Interface_Desc->frame_info[i][j].height; // 设置分辨率高度
                while ((ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &fival)) == 0) 
                {
		            if (fival.type == V4L2_FRMIVAL_TYPE_DISCRETE)  // 仅处理离散帧率
                    {      
                        Interface_Desc->frame_info[i][j].NumFPS++;  // 增加帧率计数
                        // 存储帧率值（分母即为fps值，如分母30表示30fps）
                        Interface_Desc->frame_info[i][j].FPS[fival.index] = (unsigned char)fival.discrete.denominator;
                    }
                    fival.index ++;  // 准备获取下一个帧率
                }
            }
        }
}
