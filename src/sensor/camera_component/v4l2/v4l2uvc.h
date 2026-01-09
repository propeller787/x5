/*******************************************************************************
#             uvccapture: USB UVC Video Class Snapshot Software                #
#This package work with the Logitech UVC based webcams with the mjpeg feature  #
#                                                                              #
#       Orginally Copyright (C) 2005 2006 Laurent Pinchart &&  Michel Xhaard   #
#       Modifications Copyright (C) 2006 Gabriel A. Devenyi                    #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the GNU General Public License as published by         #
# the Free Software Foundation; either version 2 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# GNU General Public License for more details.                                 #
#                                                                              #
# You should have received a copy of the GNU General Public License            #
# along with this program; if not, write to the Free Software                  #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA    #
#                                                                              #
*******************************************************************************/
#include <linux/videodev2.h>
/* 视频采集参数定义 */
#define NB_BUFFER 16       // 视频缓冲区数量
#define DHT_SIZE 420       // JPEG DHT（霍夫曼表）默认大小

/* 兼容性定义：如果系统头文件未定义锐度控制 */
#ifndef V4L2_CID_SHARPNESS
#define V4L2_CID_SHARPNESS			(V4L2_CID_BASE+27)  // 锐度控制ID
#endif

/* 视频输入设备结构体 */
struct vdIn {
  // 基础设备信息
  int fd;                   // 视频设备文件描述符
  char *videodevice;        // 视频设备路径（如/dev/video0）
  char *status;             // 设备状态信息
  char *pictName;           // 图片保存路径
  
  // V4L2数据结构
  struct v4l2_capability cap;  // 设备能力信息
  struct v4l2_format fmt;     // 视频格式信息
  struct v4l2_buffer buf;     // 视频缓冲区元数据
  struct v4l2_requestbuffers rb; // 缓冲区请求信息
  
  // 内存管理
  void *mem[NB_BUFFER];        // 内存映射指针数组
  unsigned char *tmpbuffer;    // 临时数据缓冲区
  unsigned char *framebuffer;  // 最终帧数据缓冲区
  
  // 设备状态控制
  int isstreaming;          // 流式传输状态标志（1=正在流式传输）
  int grabmethod;           // 采集方法（内存映射/用户指针等）
  int width;                // 视频宽度（像素）
  int height;               // 视频高度（像素）
  
  // 格式参数
  int formatIn;             // 输入视频格式（V4L2_PIX_FMT_XXX）
  int formatOut;            // 输出格式（用于格式转换）
  int framesizeIn;          // 输入帧大小（字节）
  
  // 控制标志
  int signalquit;           // 退出信号标志
  int toggleAvi;            // AVI录制开关
  int getPict;              // 截图获取标志
};

/* 函数声明 */
// 初始化视频设备
int init_videoIn(struct vdIn *vd, char *device, int width, int height,
                int format, int grabmethod);
// 捕获单帧数据
int uvcGrab(struct vdIn *vd);
// 关闭视频设备
int close_v4l2(struct vdIn *vd);

/* V4L2控制接口 */
int v4l2GetControl(int fd, int control);       // 获取控制值
int v4l2SetControl(int fd, int control, int value); // 设置控制值
int v4l2UpControl(int fd, int control);        // 增加控制值
int v4l2DownControl(int fd, int control);      // 减少控制值
int v4l2ToggleControl(int fd, int control);    // 切换布尔型控制
int v4l2ResetControl(int fd, int control);     // 重置控制到默认值
