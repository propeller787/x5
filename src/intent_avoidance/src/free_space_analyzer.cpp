#include "intent_avoidance/free_space_analyzer.h"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace intent_avoidance {

FreeSpaceAnalyzer::FreeSpaceAnalyzer(const Config& config)
    : config_(config)
{
}

FreeSectorList FreeSpaceAnalyzer::computeFreeSpace(
    const ObstacleSectorList& left_sectors,
    const ObstacleSectorList& right_sectors)
{
    // 合并左右扇区
    auto merged = mergeSectorLists(left_sectors, right_sectors);

    FreeSectorList free_sectors;

    // 从 -max_angle 开始迭代合并后的阻塞扇区，找间隙（=可通行扇区）
    double cursor = -config_.max_angle_rad;

    for (const auto& obs : merged) {
        // 阻塞扇区在当前游标之前则跳过
        if (obs.angle_end <= cursor) continue;

        // 阻塞扇区从游标之后开始 → 中间有可通行间隙
        if (obs.angle_start > cursor) {
            double gap_start = cursor;
            double gap_end = obs.angle_start;
            double width = gap_end - gap_start;

            if (width >= config_.min_free_sector_width_rad) {
                FreeSector fs;
                fs.angle_start = gap_start;
                fs.angle_end = gap_end;
                fs.angle_center = (gap_start + gap_end) / 2.0;
                fs.width = width;
                fs.min_clearance = std::numeric_limits<double>::max();
                free_sectors.push_back(fs);
            }
        }

        cursor = std::max(cursor, obs.angle_end);
    }

    // 最后一个阻塞扇区之后到max_angle
    if (cursor < config_.max_angle_rad) {
        double width = config_.max_angle_rad - cursor;
        if (width >= config_.min_free_sector_width_rad) {
            FreeSector fs;
            fs.angle_start = cursor;
            fs.angle_end = config_.max_angle_rad;
            fs.angle_center = (cursor + config_.max_angle_rad) / 2.0;
            fs.width = width;
            fs.min_clearance = std::numeric_limits<double>::max();
            free_sectors.push_back(fs);
        }
    }

    // 为每个可通行扇区计算最小通行距离
    for (auto& fs : free_sectors) {
        double min_clear = std::numeric_limits<double>::max();
        for (const auto& obs : merged) {
            // 阻塞扇区与可通行扇区重叠 → 用阻塞扇区的最近距离
            if (obs.angle_end > fs.angle_start && obs.angle_start < fs.angle_end) {
                min_clear = std::min(min_clear, obs.min_distance);
            }
        }
        fs.min_clearance = (min_clear == std::numeric_limits<double>::max()) ? 99.0 : min_clear;
    }

    return free_sectors;
}

NarrowPassageInfo FreeSpaceAnalyzer::detectNarrowPassage(const FreeSectorList& free_sectors)
{
    NarrowPassageInfo info;

    // 寻找覆盖正前方（0°附近）的可通行扇区
    const FreeSector* forward_sector = nullptr;
    double min_angle_diff = std::numeric_limits<double>::max();
    for (const auto& fs : free_sectors) {
        if (fs.angle_start <= 0.0 && fs.angle_end >= 0.0) {
            forward_sector = &fs;
            break;
        }
        double diff = std::min(std::abs(fs.angle_start), std::abs(fs.angle_end));
        if (diff < min_angle_diff) {
            min_angle_diff = diff;
            forward_sector = &fs;
        }
    }

    if (forward_sector == nullptr) {
        // 无可通行扇区，不是窄道
        return info;
    }

    // 边界距离：用最近障碍距离近似
    info.left_boundary_dist = forward_sector->min_clearance;
    info.right_boundary_dist = forward_sector->min_clearance;

    // 更新滑动窗口
    auto& win = boundary_window_;
    win.left_dists.push_back(info.left_boundary_dist);
    win.right_dists.push_back(info.right_boundary_dist);

    while (static_cast<int>(win.left_dists.size()) > config_.narrow_stability_frames) {
        win.left_dists.pop_front();
    }
    while (static_cast<int>(win.right_dists.size()) > config_.narrow_stability_frames) {
        win.right_dists.pop_front();
    }

    if (static_cast<int>(win.left_dists.size()) < config_.narrow_stability_frames) {
        return info;  // 数据不足
    }

    // 计算方差
    auto calc_variance = [](const std::deque<double>& d) {
        double mean = std::accumulate(d.begin(), d.end(), 0.0) / d.size();
        double var = 0.0;
        for (double v : d) {
            var += (v - mean) * (v - mean);
        }
        return var / d.size();
    };

    double left_var = calc_variance(win.left_dists);
    double right_var = calc_variance(win.right_dists);

    // 边界稳定判据：方差 < 0.01
    constexpr double kStabilityThreshold = 0.01;
    bool stable = (left_var < kStabilityThreshold) && (right_var < kStabilityThreshold);

    if (!stable) return info;

    // 计算平均通道宽度
    double avg_left = std::accumulate(win.left_dists.begin(), win.left_dists.end(), 0.0) / win.left_dists.size();
    double avg_right = std::accumulate(win.right_dists.begin(), win.right_dists.end(), 0.0) / win.right_dists.size();
    double passage_width = avg_left + avg_right;

    double min_width = config_.robot_width * config_.narrow_ratio_min;
    double max_width = config_.robot_width * config_.narrow_ratio_max;

    if (passage_width >= min_width && passage_width <= max_width) {
        info.is_narrow = true;
        info.passage_width = passage_width;
        info.left_boundary_dist = avg_left;
        info.right_boundary_dist = avg_right;
        info.center_offset = (avg_right - avg_left) / 2.0;  // +偏左, -偏右
    }

    return info;
}

ObstacleSectorList FreeSpaceAnalyzer::mergeSectorLists(
    const ObstacleSectorList& a, const ObstacleSectorList& b)
{
    ObstacleSectorList merged;
    merged.reserve(a.size() + b.size());
    merged.insert(merged.end(), a.begin(), a.end());
    merged.insert(merged.end(), b.begin(), b.end());

    if (merged.empty()) return merged;

    // 按起始角排序
    std::sort(merged.begin(), merged.end(),
        [](const ObstacleSector& lhs, const ObstacleSector& rhs) {
            return lhs.angle_start < rhs.angle_start;
        });

    // 合并重叠或相邻扇区
    ObstacleSectorList result;
    result.push_back(merged[0]);

    for (size_t i = 1; i < merged.size(); ++i) {
        auto& last = result.back();
        const auto& cur = merged[i];

        if (cur.angle_start <= last.angle_end) {
            // 重叠 → 扩展
            last.angle_end = std::max(last.angle_end, cur.angle_end);
            last.min_distance = std::min(last.min_distance, cur.min_distance);
        } else {
            result.push_back(cur);
        }
    }

    return result;
}

}  // namespace intent_avoidance
