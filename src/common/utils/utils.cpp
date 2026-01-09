#include "utils.h"
#include <chrono>
#include <vector>

namespace utils {

double legalizeAngle(double angle)
{
    while (angle < 0) {
        angle += 360;
    }

    while (angle >= 360) {
        angle -= 360;
    }

    return angle;
}

double angleDiff(double a, double b)
{
    double diff = 0.0;

    do {
        diff = a - b + 180;
        if (diff < 0) {
            diff = diff + 360 - 180;
        } else if (diff > 360) {
            diff = diff - 360 - 180;
        } else {
            diff = diff - 180;
        }
    } while (0);

    return diff;
}

double angleAverage(std::list<double> angles)
{
    std::vector<double> angle(angles.begin(), angles.end());

    double diff = 0;
    double last = angle[0];
    double sum = angle[0];

    for (size_t i = 1; i < angle.size(); i++) {
        diff = angleDiff(angle[i], angle[i - 1]);
        last += diff;
        sum += last;
    }

    return legalizeAngle(sum / angle.size());
}

uint64_t format_time()
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();

    // 将时间点转换为自纪元以来的持续时间（单位为毫秒）
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());

    // 获取持续时间的计数并返回
    return duration.count();
}

}