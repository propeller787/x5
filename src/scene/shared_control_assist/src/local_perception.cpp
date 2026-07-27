#include "shared_control_assist/local_perception.hpp"

#include <algorithm>
#include <cmath>

#include "shared_control_assist/math_utils.hpp"

namespace shared_control_assist {

LocalPerception::LocalPerception(const AssistParams& params)
    : params_(params)
{
}

ObstacleDistance LocalPerception::computeObstacleDistance(const std::vector<Point2D>& points) const
{
    ObstacleDistance dist;
    dist.valid = !points.empty();

    const double half_width = math::halfWidth(params_);
    const double front_corridor_half_width = half_width + params_.required_side_margin;
    const double side_x_min = params_.rear_edge - params_.rear_slow_distance;
    const double side_x_max = params_.front_edge + params_.front_slow_distance;
    const double forward_clearance_x_max = std::min(params_.door_roi_x_max, params_.front_edge + 1.20);

    for (const auto& p : points) {
        if (math::isInsideFootprint(params_, p.x, p.y)) {
            continue;
        }

        if (p.x >= params_.front_edge && std::abs(p.y) <= front_corridor_half_width) {
            dist.front_min = std::min(dist.front_min, p.x - params_.front_edge);
            ++dist.front_points;
        }
        if (p.x <= params_.rear_edge && std::abs(p.y) <= front_corridor_half_width) {
            dist.rear_min = std::min(dist.rear_min, params_.rear_edge - p.x);
            ++dist.rear_points;
        }

        if (p.y >= half_width && p.x >= side_x_min && p.x <= side_x_max) {
            dist.left_min = std::min(dist.left_min, p.y - half_width);
            ++dist.left_points;
        }
        if (p.y <= -half_width && p.x >= side_x_min && p.x <= side_x_max) {
            dist.right_min = std::min(dist.right_min, -half_width - p.y);
            ++dist.right_points;
        }

        if (p.x >= params_.front_edge && p.y >= half_width) {
            dist.front_left_min = std::min(
                dist.front_left_min,
                std::hypot(p.x - params_.front_edge, p.y - half_width));
        }
        if (p.x >= params_.front_edge && p.y <= -half_width) {
            dist.front_right_min = std::min(
                dist.front_right_min,
                std::hypot(p.x - params_.front_edge, p.y + half_width));
        }
        if (p.x <= params_.rear_edge && p.y >= half_width) {
            dist.rear_left_min = std::min(
                dist.rear_left_min,
                std::hypot(p.x - params_.rear_edge, p.y - half_width));
        }
        if (p.x <= params_.rear_edge && p.y <= -half_width) {
            dist.rear_right_min = std::min(
                dist.rear_right_min,
                std::hypot(p.x - params_.rear_edge, p.y + half_width));
        }

        if (p.x >= 0.0 && p.x <= forward_clearance_x_max) {
            if (p.y > half_width) {
                dist.left_clearance = std::min(dist.left_clearance, p.y - half_width);
            } else if (p.y < -half_width) {
                dist.right_clearance = std::min(dist.right_clearance, -half_width - p.y);
            }
        }
    }

    dist.min_clearance = std::min(dist.left_clearance, dist.right_clearance);
    dist.front_stop = dist.front_points >= params_.min_valid_points_for_stop &&
        dist.front_min <= params_.front_stop_distance;
    dist.rear_stop = dist.rear_points >= params_.min_valid_points_for_stop &&
        dist.rear_min <= params_.rear_stop_distance;
    dist.side_stop =
        (dist.left_points >= params_.min_valid_points_for_stop && dist.left_min <= params_.side_stop_distance) ||
        (dist.right_points >= params_.min_valid_points_for_stop && dist.right_min <= params_.side_stop_distance);

    return dist;
}

nav_msgs::msg::OccupancyGrid LocalPerception::buildLocalGrid(
    const std::vector<Point2D>& points,
    const rclcpp::Time& stamp) const
{
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = stamp;
    grid.header.frame_id = params_.base_frame;
    grid.info.resolution = params_.grid_resolution;
    grid.info.width = static_cast<std::uint32_t>(
        std::ceil((params_.grid_x_max - params_.grid_x_min) / params_.grid_resolution));
    grid.info.height = static_cast<std::uint32_t>(
        std::ceil((params_.grid_y_max - params_.grid_y_min) / params_.grid_resolution));
    grid.info.origin.position.x = params_.grid_x_min;
    grid.info.origin.position.y = params_.grid_y_min;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<std::size_t>(grid.info.width) * grid.info.height, 0);

    const int inflation_cells = std::max(0, static_cast<int>(std::ceil(params_.grid_inflation / params_.grid_resolution)));
    const int width = static_cast<int>(grid.info.width);
    const int height = static_cast<int>(grid.info.height);

    for (const auto& p : points) {
        if (p.x < params_.grid_x_min || p.x >= params_.grid_x_max ||
            p.y < params_.grid_y_min || p.y >= params_.grid_y_max) {
            continue;
        }

        const int gx = static_cast<int>((p.x - params_.grid_x_min) / params_.grid_resolution);
        const int gy = static_cast<int>((p.y - params_.grid_y_min) / params_.grid_resolution);

        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                if (dx * dx + dy * dy > inflation_cells * inflation_cells) {
                    continue;
                }
                const int ix = gx + dx;
                const int iy = gy + dy;
                if (ix < 0 || ix >= width || iy < 0 || iy >= height) {
                    continue;
                }
                grid.data[static_cast<std::size_t>(iy * width + ix)] = 100;
            }
        }
    }

    return grid;
}

}  // namespace shared_control_assist
