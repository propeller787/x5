#pragma once

#include <string>
#include <vector>

#include <rclcpp/time.hpp>

#include "shared_control_assist/types.hpp"

namespace shared_control_assist {

// 门洞检测输出。
//
// candidate:
//   本周期门候选。如果 detected=false，表示没有可用候选。
// reason:
//   面向调试的人类可读原因，例如：
//   - "door_candidate_ready"       候选稳定且置信度足够。
//   - "door_candidate_warming_up"  已检测到，但稳定帧数/置信度还在积累。
//   - "holding_recent_door_candidate" 短时丢失，正在保持上一候选。
//   - "no_door_candidate"         没有候选。
struct DoorDetectionResult {
    DoorCandidate candidate;
    std::string reason;
};

// 轻量门洞检测器。
//
// 算法顺序：
// 1. 最大自由间隙：
//    在前方 ROI 内取多个 x 截面，对 y 方向离散成 bin，寻找最大的可通行空隙。
//    适合门框不完整、只看到局部边界的场景，是 MVP 的主检测方法。
//
// 2. 平行墙/通道 fallback：
//    若最大自由间隙没有结果，则在 ROI 内寻找左右最近边界，估计通道宽度。
//    适合窄通道、门洞两侧延伸墙面等场景。
//
// 3. 时间稳定性：
//    连续帧门宽、中心横向偏移、角度变化都足够小，才增加 stable_count。
//    这样可以降低单帧噪声导致突然进入窄门辅助的风险。
//
// 当前 MVP 限制：
// - 尚未做真正的门框聚类或线段拟合。
// - DoorCandidate::yaw / angle_error 暂时保持 0。
// - 对斜门、大角度入门的识别仍需要后续扩展。
class DoorDetector {
public:
    explicit DoorDetector(const AssistParams& params);

    // 检测当前点云中的门洞候选。
    //
    // 输入点必须已经在 base_frame 下，且最好已经过滤掉车体 footprint 内部点。
    // 该函数会更新内部稳定性状态 stable_door_ 和 last_candidate_time_。
    //
    // 线程约定：
    // - 当前节点只在控制定时器线程中调用该函数。
    // - 若未来改成多线程调用，需要给内部状态加锁或将状态外置。
    DoorDetectionResult detect(const std::vector<Point2D>& points, const rclcpp::Time& now);

private:
    // 主检测方法：在多个 x 截面上寻找 y 方向最大连续自由区间。
    //
    // 输出 source=3(MAX_FREE_GAP)。
    // 置信度由宽度合法性、左右边界是否存在、中心偏移、距离等规则组合得到。
    DoorCandidate detectMaxFreeGap(const std::vector<Point2D>& points) const;

    // 降级检测方法：估计左右墙/边界之间的通道宽度。
    //
    // 输出 source=2(PARALLEL_WALLS)。
    // 当前实现没有线拟合，只取左右侧最近 y 边界，因此适合作为低复杂度 fallback。
    DoorCandidate detectParallelWallGap(const std::vector<Point2D>& points) const;

    // 参数引用，生命周期由 SharedControlAssistNode 保证。
    const AssistParams& params_;

    // 上一帧稳定候选。用于：
    // - 判断当前候选是否与上一帧足够接近。
    // - 候选短时丢失时保持一小段时间，避免控制状态突然抖动。
    DoorCandidate stable_door_;

    // 上次检测到候选的时间。用于 candidate_lost_timeout 判断。
    rclcpp::Time last_candidate_time_{0, 0, RCL_ROS_TIME};
};

}  // namespace shared_control_assist
