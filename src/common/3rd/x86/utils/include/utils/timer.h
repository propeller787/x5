#pragma once

#include "string"
#include <list>
#include <muduo/base/Timestamp.h>
#include <muduo/net/EventLoop.h>
#include <muduo/net/EventLoopThread.h>

namespace utils {

class Timer {
public:
    Timer(muduo::net::EventLoop* loop);

    // 开始计时器，指定秒数后触发事件
    void startTimer(double seconds, const muduo::net::TimerCallback& callback);

private:
    muduo::net::EventLoop* loop_; // 事件循环
};

}