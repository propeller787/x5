#include "shared_control_assist/door_detector.hpp"

#include <algorithm>
#include <cmath>

#include "shared_control_assist/math_utils.hpp"

namespace shared_control_assist {

DoorDetector::DoorDetector(const AssistParams& params)
    : params_(params)
{
}

DoorDetectionResult DoorDetector::detect(const std::vector<Point2D>& points, const rclcpp::Time& now)
{
    DoorCandidate raw = detectMaxFreeGap(points);
    if (!raw.detected) {
        raw = detectParallelWallGap(points);
    }

    if (!raw.detected) {
        if (stable_door_.detected && math::timeAgeSeconds(now, last_candidate_time_) <= params_.candidate_lost_timeout) {
            DoorCandidate held = stable_door_;
            held.confidence = std::max(0.0, held.confidence - 0.15);
            held.reason = 3;
            return {held, "holding_recent_door_candidate"};
        }

        stable_door_ = DoorCandidate();
        return {raw, "no_door_candidate"};
    }

    const bool stable_with_previous = stable_door_.detected &&
        math::closeEnough(raw.width, stable_door_.width, 0.08) &&
        math::closeEnough(raw.center_y, stable_door_.center_y, 0.06) &&
        math::closeEnough(raw.yaw, stable_door_.yaw, 8.0 * math::kPi / 180.0);
    raw.stable_count = stable_with_previous ? std::min(stable_door_.stable_count + 1, params_.door_stable_frame_count) : 1;
    const double temporal_score = math::clamp(
        static_cast<double>(raw.stable_count) / static_cast<double>(params_.door_stable_frame_count),
        0.0,
        1.0);
    raw.confidence = math::clamp(raw.confidence + 0.20 * temporal_score, 0.0, 1.0);

    stable_door_ = raw;
    last_candidate_time_ = now;
    const std::string reason =
        raw.confidence >= params_.door_confidence_enter ? "door_candidate_ready" : "door_candidate_warming_up";
    return {raw, reason};
}

DoorCandidate DoorDetector::detectMaxFreeGap(const std::vector<Point2D>& points) const
{
    DoorCandidate best;
    double best_score = -1.0;

    const double passable_width = params_.vehicle_width + 2.0 * params_.required_side_margin;
    const int bin_count = std::max(
        1,
        static_cast<int>(std::ceil((params_.door_roi_y_max - params_.door_roi_y_min) / params_.door_bin_resolution)));

    for (double x = params_.door_roi_x_min; x <= params_.door_roi_x_max; x += params_.door_x_sample_step) {
        std::vector<bool> occupied(static_cast<std::size_t>(bin_count), false);
        int occupied_count = 0;

        for (const auto& p : points) {
            if (p.x < params_.door_roi_x_min || p.x > params_.door_roi_x_max ||
                p.y < params_.door_roi_y_min || p.y > params_.door_roi_y_max) {
                continue;
            }
            if (std::abs(p.x - x) > params_.door_x_window) {
                continue;
            }

            const int bin = static_cast<int>((p.y - params_.door_roi_y_min) / params_.door_bin_resolution);
            if (bin < 0 || bin >= bin_count) {
                continue;
            }
            if (!occupied[static_cast<std::size_t>(bin)]) {
                occupied[static_cast<std::size_t>(bin)] = true;
                ++occupied_count;
            }
        }

        if (occupied_count < 2) {
            continue;
        }

        int free_start = -1;
        for (int i = 0; i <= bin_count; ++i) {
            const bool is_free = i < bin_count && !occupied[static_cast<std::size_t>(i)];
            if (is_free && free_start < 0) {
                free_start = i;
            }
            if ((!is_free || i == bin_count) && free_start >= 0) {
                const int interval_start = free_start;
                const int free_end = i - 1;
                const double free_y0 = params_.door_roi_y_min + interval_start * params_.door_bin_resolution;
                const double free_y1 = params_.door_roi_y_min + (free_end + 1) * params_.door_bin_resolution;
                const bool has_right_boundary = interval_start > 0;
                const bool has_left_boundary = free_end < bin_count - 1;
                const double right_boundary_y =
                    has_right_boundary ? free_y0 - 0.5 * params_.door_bin_resolution : free_y0;
                const double left_boundary_y =
                    has_left_boundary ? free_y1 + 0.5 * params_.door_bin_resolution : free_y1;
                const double width = left_boundary_y - right_boundary_y;
                const double center_y = 0.5 * (right_boundary_y + left_boundary_y);

                free_start = -1;

                if (width < passable_width || width < params_.min_door_width || width > params_.max_door_width) {
                    continue;
                }
                if (std::abs(center_y) > 0.50) {
                    continue;
                }

                const double boundary_score = (has_left_boundary && has_right_boundary) ? 1.0 : 0.45;
                const double width_score = math::clamp(
                    (width - params_.min_door_width) / std::max(0.01, params_.max_door_width - params_.min_door_width),
                    0.0,
                    1.0);
                const double center_score = math::clamp(1.0 - std::abs(center_y) / 0.50, 0.0, 1.0);
                const double range_score = math::clamp(
                    1.0 - (x - params_.door_roi_x_min) /
                        std::max(0.01, params_.door_roi_x_max - params_.door_roi_x_min),
                    0.0,
                    1.0);
                const double score = 0.30 + 0.25 * boundary_score + 0.20 * center_score +
                    0.15 * width_score + 0.10 * range_score;

                if (score <= best_score) {
                    continue;
                }

                DoorCandidate candidate;
                candidate.detected = true;
                candidate.confidence = math::clamp(score, 0.0, 0.80);
                candidate.center_x = x;
                candidate.center_y = center_y;
                candidate.yaw = 0.0;
                candidate.width = width;
                candidate.distance = x;
                candidate.left_boundary_x = x;
                candidate.left_boundary_y = left_boundary_y;
                candidate.right_boundary_x = x;
                candidate.right_boundary_y = right_boundary_y;
                candidate.left_clearance = left_boundary_y - math::halfWidth(params_);
                candidate.right_clearance = -right_boundary_y - math::halfWidth(params_);
                candidate.angle_error = 0.0;
                candidate.lateral_error = center_y;
                candidate.source = 3;
                candidate.reason = 0;

                best = candidate;
                best_score = score;
            }
        }
    }

    return best;
}

DoorCandidate DoorDetector::detectParallelWallGap(const std::vector<Point2D>& points) const
{
    const double half_width = math::halfWidth(params_);
    const double passable_width = params_.vehicle_width + 2.0 * params_.required_side_margin;

    double left_y = math::kInf;
    double right_y = -math::kInf;
    double left_x_sum = 0.0;
    double right_x_sum = 0.0;
    int left_count = 0;
    int right_count = 0;

    for (const auto& p : points) {
        if (p.x < params_.door_roi_x_min || p.x > params_.door_roi_x_max ||
            p.y < params_.door_roi_y_min || p.y > params_.door_roi_y_max) {
            continue;
        }
        if (p.y > half_width + params_.required_side_margin) {
            if (p.y < left_y) {
                left_y = p.y;
            }
            left_x_sum += p.x;
            ++left_count;
        } else if (p.y < -half_width - params_.required_side_margin) {
            if (p.y > right_y) {
                right_y = p.y;
            }
            right_x_sum += p.x;
            ++right_count;
        }
    }

    DoorCandidate candidate;
    if (left_count < params_.min_valid_points_for_stop || right_count < params_.min_valid_points_for_stop) {
        return candidate;
    }

    const double width = left_y - right_y;
    if (width < passable_width || width < params_.min_door_width || width > params_.max_door_width) {
        candidate.reason = 1;
        return candidate;
    }

    const double center_y = 0.5 * (left_y + right_y);
    const double center_x = 0.5 * (left_x_sum / left_count + right_x_sum / right_count);

    candidate.detected = true;
    candidate.confidence = 0.60;
    candidate.center_x = math::clamp(center_x, params_.door_roi_x_min, params_.door_roi_x_max);
    candidate.center_y = center_y;
    candidate.yaw = 0.0;
    candidate.width = width;
    candidate.distance = candidate.center_x;
    candidate.left_boundary_x = candidate.center_x;
    candidate.left_boundary_y = left_y;
    candidate.right_boundary_x = candidate.center_x;
    candidate.right_boundary_y = right_y;
    candidate.left_clearance = left_y - half_width;
    candidate.right_clearance = -right_y - half_width;
    candidate.angle_error = 0.0;
    candidate.lateral_error = center_y;
    candidate.source = 2;
    candidate.reason = 0;
    return candidate;
}

}  // namespace shared_control_assist
