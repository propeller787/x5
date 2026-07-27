#pragma once

#include "intent_avoidance/types.h"

#include <deque>
#include <mutex>

namespace intent_avoidance {

// 阻塞扇区 → 可通行扇区 + 窄道检测
class FreeSpaceAnalyzer {
public:
    struct Config {
        double min_free_sector_width_rad = 0.349;   // ≈20°，小于此宽度的扇区丢弃
        double robot_width = 0.70;                   // 车宽(m)
        double narrow_ratio_min = 1.2;               // 窄道最小通道/车宽比
        double narrow_ratio_max = 2.5;               // 窄道最大通道/车宽比
        int narrow_stability_frames = 15;            // 窄道边界稳定帧数
        double max_angle_rad = 1.57;                 // 考虑的最大角度范围(rad, ≈90°)
    };

    explicit FreeSpaceAnalyzer(const Config& config);

    // 输入左右雷达的阻塞扇区，计算可通行扇区
    FreeSectorList computeFreeSpace(
        const ObstacleSectorList& left_sectors,
        const ObstacleSectorList& right_sectors);

    // 检测窄道
    NarrowPassageInfo detectNarrowPassage(const FreeSectorList& free_sectors);

private:
    // 合并两个扇区列表并排序
    static ObstacleSectorList mergeSectorLists(
        const ObstacleSectorList& a, const ObstacleSectorList& b);

    Config config_;

    // 窄道检测的边界滑动窗口
    struct BoundaryWindow {
        std::deque<double> left_dists;
        std::deque<double> right_dists;
    };
    BoundaryWindow boundary_window_;
};

}  // namespace intent_avoidance
