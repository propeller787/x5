#include "lidar_obstacle_scantrend.h"

#include <chrono>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace {

using SteadyClock = std::chrono::steady_clock;

int64_t elapsedUs(const SteadyClock::time_point& start)
{
    return std::chrono::duration_cast<std::chrono::microseconds>(SteadyClock::now() - start).count();
}

void logCallbackTiming(
    const rclcpp::Logger& logger,
    const std::string& frame_id,
    int64_t callback_total_us,
    int64_t segmenter_us,
    int64_t pcl_debug_us,
    int64_t pcl_obstacle_us,
    int64_t publish_debug_us,
    int64_t publish_obstacle_us)
{
    RCLCPP_INFO(
        logger,
        "[%s] callback timing total=%.2fms | segmenter=%.2f pcl_debug=%.2f "
        "pcl_obstacle=%.2f publish_debug=%.2f publish_obstacle=%.2f",
        frame_id.c_str(),
        callback_total_us / 1000.0,
        segmenter_us / 1000.0,
        pcl_debug_us / 1000.0,
        pcl_obstacle_us / 1000.0,
        publish_debug_us / 1000.0,
        publish_obstacle_us / 1000.0);
}

void logScanTrendTiming(
    const rclcpp::Logger& logger, const std::string& frame_id, const ScanTrendTiming& timing)
{
    RCLCPP_INFO(
        logger,
        "[%s] scantrend timing n=%zu pipeline=%d total=%.2fms | "
        "transform=%.2f sort=%.2f label=%.2f propagate=%.2f pos_slab=%.2f "
        "count_neg=%.2f grate=%.2f build=%.2f inject=%.2f",
        frame_id.c_str(),
        timing.n_points,
        timing.run_ground_pipeline ? 1 : 0,
        timing.total_us / 1000.0,
        timing.transform_us / 1000.0,
        timing.sort_us / 1000.0,
        timing.label_initial_us / 1000.0,
        timing.propagate_us / 1000.0,
        timing.pos_slab_us / 1000.0,
        timing.count_neg_us / 1000.0,
        timing.grate_us / 1000.0,
        timing.build_output_us / 1000.0,
        timing.inject_virtual_us / 1000.0);
}

Eigen::Matrix4d transformToMatrix(const geometry_msgs::msg::Transform& transform)
{
    Eigen::Quaterniond q(
        transform.rotation.w,
        transform.rotation.x,
        transform.rotation.y,
        transform.rotation.z);
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = q.normalized().toRotationMatrix();
    matrix(0, 3) = transform.translation.x;
    matrix(1, 3) = transform.translation.y;
    matrix(2, 3) = transform.translation.z;
    return matrix;
}

bool lookupTransformMatrix(
    const tf2_ros::Buffer& buffer,
    const std::string& target_frame,
    const std::string& source_frame,
    const rclcpp::Time& time,
    Eigen::Matrix4d& out_matrix)
{
    try {
        const geometry_msgs::msg::TransformStamped transform = buffer.lookupTransform(
            target_frame,
            source_frame,
            time,
            tf2::durationFromSec(0.1));
        out_matrix = transformToMatrix(transform.transform);
        return true;
    } catch (const tf2::TransformException&) {
        return false;
    }
}

PointT transformPoint(const PointT& point, const Eigen::Matrix4d& matrix)
{
    const Eigen::Vector4d v(point.x, point.y, point.z, 1.0);
    const Eigen::Vector4d out = matrix * v;
    return PointT(
        static_cast<float>(out.x()),
        static_cast<float>(out.y()),
        static_cast<float>(out.z()));
}

bool isValidScanRange(float range, const sensor_msgs::msg::LaserScan& scan)
{
    if (!std::isfinite(range)) {
        return false;
    }
    if (range <= scan.range_min || range >= scan.range_max) {
        return false;
    }
    return true;
}

constexpr float kTiltMaxObstacleZ = 0.28f;

PointCloud::Ptr horizScanToBaseLinkCloud(
    const sensor_msgs::msg::LaserScan& scan,
    const tf2_ros::Buffer& buffer,
    const std::string& target_frame,
    float min_x,
    bool* ok)
{
    auto cloud = std::make_shared<PointCloud>();
    if (ok != nullptr) {
        *ok = false;
    }

    Eigen::Matrix4d source_to_target = Eigen::Matrix4d::Identity();
    const rclcpp::Time stamp(scan.header.stamp);
    if (!lookupTransformMatrix(buffer, target_frame, scan.header.frame_id, stamp, source_to_target)) {
        return cloud;
    }

    if (ok != nullptr) {
        *ok = true;
    }

    cloud->points.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const float range = scan.ranges[i];
        if (!isValidScanRange(range, scan)) {
            continue;
        }

        const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
        const PointT point_laser(
            range * std::cos(angle),
            range * std::sin(angle),
            0.f);
        const PointT point_base = transformPoint(point_laser, source_to_target);
        if (point_base.x > min_x) {
            cloud->points.push_back(point_base);
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

// 检查点坐标是否为有限值
bool isFinitePoint(const ScanPoint& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

// 保留有效 scan 角度（过滤掉无效扇区）
bool useScanAngle(float angle)
{
    return (angle >= 0.f && angle <= static_cast<float>(M_PI / 2.0)) ||
           (angle > static_cast<float>(3.0 * M_PI / 2.0) && angle <= static_cast<float>(2.0 * M_PI)) ||
           (angle < 0.f && angle >= static_cast<float>(-M_PI / 2.0));
}

// 判断两点 x 距离是否在 pos_slab 条带宽度内
bool inXGrid(float x_i, float x_j, float grid_x_max)
{
    return std::abs(x_i - x_j) <= grid_x_max * 0.5f;
}

// grate 滤除时可用于统计邻域地面数的点（Ground 或低 z 正障碍）
bool isGrateNeighborCandidate(const ScanPoint& point, const ScanTrendParams& params)
{
    if (point.label == PointLabel::Ground) {
        return true;
    }
    return point.label == PointLabel::Obstacle && !point.negative_obstacle &&
           std::abs(point.z) < params.positive_dz_min;
}

// 点 x 是否落在侧向 blind spot 线段 x 范围（含 margin）内
bool isInSideSegmentXRange(
    const ScanPoint& point,
    const BlindSpotSideRegion& region,
    float x_margin)
{
    const float xmin = std::min(region.start_x, region.end_x) - x_margin;
    const float xmax = std::max(region.start_x, region.end_x) + x_margin;
    return point.x >= xmin && point.x <= xmax;
}

// 计算点到 side_blind_spot 标定线段的垂距；投影落在线段外返回 false
bool distanceToSideLineSegment(
    const ScanPoint& point, const BlindSpotSideRegion& region, float& out_dist)
{
    const Eigen::Vector3f start(region.start_x, region.start_y, region.start_z);
    const Eigen::Vector3f end(region.end_x, region.end_y, region.end_z);
    const Eigen::Vector3f p(point.x, point.y, point.z);
    const Eigen::Vector3f direction = end - start;
    const float length = direction.norm();
    if (length < 1e-3f) {
        return false;
    }

    const Eigen::Vector3f unit_dir = direction / length;
    const float proj_len = (p - start).dot(unit_dir);
    if (proj_len < 0.f || proj_len > length) {
        return false;
    }

    const Eigen::Vector3f nearest = start + proj_len * unit_dir;
    out_dist = (p - nearest).norm();
    return true;
}

// 点是否在标定线窄走廊内（投影在线段上且垂距 ≤ tol）
bool isInSideBlindSpotRegion(
    const ScanPoint& point,
    const BlindSpotSideRegion& region,
    float line_dist_tol)
{
    float dist = 0.f;
    if (!distanceToSideLineSegment(point, region, dist)) {
        return false;
    }
    return dist <= line_dist_tol;
}

// 统计侧向 x 带内已标负障碍物的点数（用于虚拟障碍注入触发）
int countNegInSideRegion(
    const std::vector<ScanPoint>& points,
    const BlindSpotSideRegion& region,
    const ScanTrendParams& params)
{
    int count = 0;
    for (const auto& point : points) {
        if (point.label != PointLabel::Obstacle || !point.negative_obstacle) {
            continue;
        }
        if (!isInSideSegmentXRange(point, region, params.blind_spot_side_x_margin)) {
            continue;
        }
        ++count;
    }
    return count;
}

// 求 float 序列中位数
float medianValue(std::vector<float> values)
{
    if (values.empty()) {
        return 0.f;
    }
    std::sort(values.begin(), values.end());
    return values[values.size() / 2];
}

// 求 float 序列算术平均
float meanValue(const std::vector<float>& values)
{
    if (values.empty()) {
        return 0.f;
    }
    float sum = 0.f;
    for (float v : values) {
        sum += v;
    }
    return sum / static_cast<float>(values.size());
}

// 虚拟 blind spot 障碍物 y 坐标限幅（随左右雷达 start_y 符号）
float clampBlindSpotVirtualY(float y, float start_y, float y_min)
{
    if (start_y > 0.f) {
        return std::max(y, y_min);
    }
    return std::min(y, -y_min);
}

// 计算 scan 上两点间的有符号坡度 dz/dx；|dx| 过小时返回 false
bool computeScanSlope(
    const ScanPoint& from, const ScanPoint& to, float min_dx, float& out_slope)
{
    const float dx = to.x - from.x;
    if (std::abs(dx) < min_dx) {
        return false;
    }
    out_slope = (to.z - from.z) / dx;
    return true;
}

// 两坡度是否接近（绝对差或相对差满足其一）
bool slopesAreSimilar(float a, float b, float abs_tol, float ratio_tol)
{
    if (a > 0.01f || b > 0.01f) {
        return false;
    }
    const float diff = std::abs(a - b);
    if (diff <= abs_tol) {
        return true;
    }
    const float denom = std::max({std::abs(a), std::abs(b), 1e-3f});
    return diff / denom <= ratio_tol;
}

// 跳变前锚点 edge_i 回看 history 基线，任一累计斜率与跳变边接近则视为缓坡延续
bool isGradualRampBackwardConsistency(
    const std::vector<ScanPoint>& points, size_t edge_i, const ScanTrendParams& params)
{
    const size_t anchor = edge_i;
    const size_t after = edge_i + 1;
    if (after >= points.size()) {
        return false;
    }

    float slope_jump = 0.f;
    if (!computeScanSlope(
            points[anchor], points[after], params.ramp_min_dx, slope_jump)) {
        return false;
    }
    if (slope_jump > 0.f) {
        return false;
    }

    for (int k = 1; k < params.z_ref_window; ++k) {
        if (anchor < static_cast<size_t>(k)) {
            break;
        }
        const size_t ref = anchor - static_cast<size_t>(k);
        float slope_k = 0.f;
        if (!computeScanSlope(points[ref], points[anchor], params.ramp_min_dx, slope_k)) {
            continue;
        }
        if (slopesAreSimilar(
                slope_k, slope_jump, params.ramp_slope_abs_tol, params.ramp_slope_ratio_tol)) {
            return true;
        }
    }

    return false;
}

// 跳变边为缓坡延续（非台阶/坑口）时返回 true，isZJumpEdge 将不触发
bool isGradualRampEdge(const std::vector<ScanPoint>& points, size_t edge_i, const ScanTrendParams& params)
{
    if (edge_i + 1 >= points.size()) {
        return false;
    }
    return isGradualRampBackwardConsistency(points, edge_i, params);
}

// 标为负障碍物（不覆盖已标正障碍）
void markNegativeObstacle(ScanPoint& point)
{
    if (point.label == PointLabel::Obstacle && !point.negative_obstacle) {
        return;
    }
    point.label = PointLabel::Obstacle;
    point.negative_obstacle = true;
}

// 标为凸台/cliff 正障碍物
void markCliffObstacle(ScanPoint& point)
{
    point.label = PointLabel::Obstacle;
    point.negative_obstacle = false;
}

// 标为地面点
void markGroundPoint(ScanPoint& point)
{
    point.label = PointLabel::Ground;
    point.negative_obstacle = false;
}

// 是否为侧向标定线上的 ground 种子（x 在带内、投影在线段、垂距 ≤ tol）
bool isSideGroundSeed(
    const ScanPoint& point, const BlindSpotSideRegion& region, const ScanTrendParams& params)
{
    if (!isInSideSegmentXRange(point, region, params.blind_spot_side_x_margin)) {
        return false;
    }
    float dist = 0.f;
    if (!distanceToSideLineSegment(point, region, dist)) {
        return false;
    }
    return dist <= params.blind_spot_line_dist_tol;
}

// 合并 scan 序上连续的 ground 种子为若干 [begin, end] 段
void collectSideGroundSeedSegments(
    const std::vector<ScanPoint>& points,
    const BlindSpotSideRegion& region,
    const ScanTrendParams& params,
    std::vector<std::pair<size_t, size_t>>& segments)
{
    segments.clear();
    for (size_t i = 0; i < points.size(); ++i) {
        if (!isSideGroundSeed(points[i], region, params)) {
            continue;
        }
        if (segments.empty() || i > segments.back().second + 1) {
            segments.emplace_back(i, i);
        } else {
            segments.back().second = i;
        }
    }
}

// 从 seed 段列表取全局最小/最大 ground 种子 index
bool findSideGroundSeedIndexRange(
    const std::vector<std::pair<size_t, size_t>>& segments, size_t& min_idx, size_t& max_idx)
{
    if (segments.empty()) {
        return false;
    }
    min_idx = segments.front().first;
    max_idx = segments.front().second;
    for (const auto& segment : segments) {
        min_idx = std::min(min_idx, segment.first);
        max_idx = std::max(max_idx, segment.second);
    }
    return true;
}

// scan 相邻点 Δx 是否断链（低装雷达：换支常伴随 ground→obstacle）
bool isScanStepDxBreak(
    const std::vector<ScanPoint>& points, size_t edge_i, const ScanTrendParams& params)
{
    if (edge_i + 1 >= points.size()) {
        return false;
    }
    return std::abs(points[edge_i + 1].x - points[edge_i].x) > params.max_expand_dx;
}

// 相邻 scan 点是否 z 向突变（不含 Δx 判定，Δx 由 isScanStepDxBreak 单独处理）
bool isZJumpEdge(const std::vector<ScanPoint>& points, size_t edge_i, const ScanTrendParams& params)
{
    if (edge_i + 1 >= points.size()) {
        return false;
    }
    const float dz = points[edge_i + 1].z - points[edge_i].z;
    if (std::abs(dz) < params.negative_dz_min) {
        return false;
    }
    // 如果满足缓坡约束，则不认为是 z 突变边
    if (isGradualRampEdge(points, edge_i, params)) {
        return false;
    }
    return true;
}

void markObstacleInSweep(ScanPoint& point, bool negative)
{
    if (negative) {
        markNegativeObstacle(point);
    } else {
        markCliffObstacle(point);
    }
}

// 从 max ground 种子沿 index 增大方向传播：无突变标 Ground；Δx 断链或 z 突变后余段全标 Obstacle
void propagateGroundForward(
    std::vector<ScanPoint>& points, size_t start_idx, const ScanTrendParams& params)
{
    if (start_idx + 1 >= points.size()) {
        return;
    }

    bool obstacle_mode = false;
    bool obstacle_negative = true;
    float ground_z = points[start_idx].z;

    for (size_t i = start_idx; i + 1 < points.size(); ++i) {
        const size_t next = i + 1;
        const float dz = points[next].z - points[i].z;

        if (!obstacle_mode) {
            if (isScanStepDxBreak(points, i, params)) {
                obstacle_mode = true;
                obstacle_negative = points[next].z < ground_z;
                if (obstacle_negative) {
                    markObstacleInSweep(points[i], true);
                }
                markObstacleInSweep(points[next], obstacle_negative);
            } else if (isZJumpEdge(points, i, params)) {
                obstacle_mode = true;
                obstacle_negative = dz < 0.f;
                if (obstacle_negative) {
                    markObstacleInSweep(points[i], true);
                }
                markObstacleInSweep(points[next], obstacle_negative);
            } else {
                markGroundPoint(points[next]);
                ground_z = points[next].z;
            }
        } else {
            markObstacleInSweep(points[next], obstacle_negative);
        }
    }
}

// 从 min ground 种子沿 index 减小方向传播；逻辑同 propagateGroundForward
void propagateGroundBackward(
    std::vector<ScanPoint>& points, size_t start_idx, const ScanTrendParams& params)
{
    if (start_idx == 0) {
        return;
    }

    bool obstacle_mode = false;
    bool obstacle_negative = true;
    float ground_z = points[start_idx].z;

    for (size_t i = start_idx; i > 0; --i) {
        const size_t prev = i - 1;
        const float dz = points[prev].z - points[i].z;

        if (!obstacle_mode) {
            if (isScanStepDxBreak(points, prev, params)) {
                obstacle_mode = true;
                obstacle_negative = points[prev].z < ground_z;
                if (obstacle_negative) {
                    markObstacleInSweep(points[i], true);
                }
                markObstacleInSweep(points[prev], obstacle_negative);
            } else if (isZJumpEdge(points, prev, params)) {
                obstacle_mode = true;
                obstacle_negative = dz < 0.f;
                if (obstacle_negative) {
                    markObstacleInSweep(points[i], true);
                }
                markObstacleInSweep(points[prev], obstacle_negative);
            } else {
                markGroundPoint(points[prev]);
                ground_z = points[prev].z;
            }
        } else {
            markObstacleInSweep(points[prev], obstacle_negative);
        }
    }
}

}  // namespace

// 从 txt 加载雷达外参（tz, ty, tx, rz, ry, rx；第 7 行 sort_angle_deg 若存在则忽略）
bool loadExtrinsicTxt(const std::string& path, LidarGroundCalib& calib)
{
    std::ifstream infile(path);
    if (!infile.is_open()) {
        return false;
    }

    std::vector<double> values;
    std::string line;
    while (std::getline(infile, line)) {
        if (!line.empty()) {
            values.push_back(std::stod(line));
        }
    }

    if (values.size() < 6) {
        return false;
    }

    calib.tz = values[0];
    calib.ty = values[1];
    calib.tx = values[2];
    calib.rz = values[3];
    calib.ry = values[4];
    calib.rx = values[5];
    return true;
}

// 从 yaml 加载地面模型参数及 side_blind_spot 标定线段
bool loadGroundYaml(const std::string& path, LidarGroundCalib& calib)
{
    YAML::Node config = YAML::LoadFile(path);
    if (!config["ground_model"]) {
        return false;
    }

    const auto& ground_model = config["ground_model"];
    if (ground_model["ranges"]) {
        const auto& ranges = ground_model["ranges"][0];
        const double x = ranges["x"].as<double>();
        const double y = ranges["y"].as<double>();
        const double det_x = ranges["det_x"].as<double>();
        const double det_y = ranges["det_y"].as<double>();
        if (std::abs(det_x) < 1e-6) {
            calib.polar_k = 100000.0;
            calib.polar_b = x;
        } else {
            calib.polar_k = det_y / det_x;
            calib.polar_b = y - calib.polar_k * x;
        }
    }

    if (ground_model["start_point_side"] && ground_model["end_point_side"]) {
        const auto& start = ground_model["start_point_side"][0];
        const auto& end = ground_model["end_point_side"][0];
        calib.side_blind_spot.start_x = start["x"].as<float>();
        calib.side_blind_spot.start_y = start["y"].as<float>();
        calib.side_blind_spot.start_z = start["z"].as<float>();
        calib.side_blind_spot.end_x = end["x"].as<float>();
        calib.side_blind_spot.end_y = end["y"].as<float>();
        calib.side_blind_spot.end_z = end["z"].as<float>();
        calib.side_blind_spot.loaded = true;
    }
    return true;
}

// 由外参计算 rotation / translation
void finalizeCalib(LidarGroundCalib& calib)
{
    calib.rotation =
        Eigen::AngleAxisd(calib.rz, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(calib.ry, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(calib.rx, Eigen::Vector3d::UnitX());
    calib.translation << 0.0, 0.0, calib.tz;
    calib.loaded = calib.tz > 1e-6;
}

ScanTrendSegmenter::ScanTrendSegmenter(ScanTrendParams params)
    : params_(params)
{
}

// 检测近距 scan 是否噪声过多（决定是否过滤近距点）
bool ScanTrendSegmenter::checkScanNoise(const sensor_msgs::msg::LaserScan& scan) const
{
    int count = params_.noise_min_count;
    for (const float range : scan.ranges) {
        if (range >= scan.range_min && range < params_.noise_check_range) {
            --count;
        }
    }
    return count == params_.noise_min_count || count < 0;
}

// LaserScan 极坐标转车体坐标系 ScanPoint 列表
std::vector<ScanPoint> ScanTrendSegmenter::transformScan(
    const sensor_msgs::msg::LaserScan& scan,
    const LidarGroundCalib& calib) const
{
    std::vector<ScanPoint> points;
    points.reserve(scan.ranges.size());

    const bool no_noise = checkScanNoise(scan);
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
        const float range = scan.ranges[i];
        if (range < scan.range_min || range > scan.range_max) {
            continue;
        }
        if (!no_noise && range < params_.noise_check_range) {
            continue;
        }

        const float angle = scan.angle_min + static_cast<float>(i) * scan.angle_increment;
        if (!useScanAngle(angle)) {
            continue;
        }

        const float local_x = range * std::cos(angle);
        const float local_y = range * std::sin(angle);

        ScanPoint point;
        point.beam_index = static_cast<uint16_t>(i);
        point.range = range;
        point.angle = angle;
        point.x = static_cast<float>(
            calib.rotation(0, 0) * local_x + calib.rotation(0, 1) * local_y + calib.translation(0));
        point.y = static_cast<float>(
            calib.rotation(1, 0) * local_x + calib.rotation(1, 1) * local_y + calib.translation(1));
        point.z = static_cast<float>(
            calib.rotation(2, 0) * local_x + calib.rotation(2, 1) * local_y + calib.translation(2));

        if (point.z >= params_.max_point_z || !isFinitePoint(point) || point.x < -1.0) {
            continue;
        }

        points.push_back(point);
    }

    return points;
}

// 按 LaserScan beam 序号排序（-π~π 全周单线雷达；与 sortAngle 序等价），必要时 reverse 使 x 递增
void ScanTrendSegmenter::sortScanPoints(std::vector<ScanPoint>& points) const
{
    std::sort(points.begin(), points.end(), [](const ScanPoint& a, const ScanPoint& b) {
        return a.beam_index < b.beam_index;
    });

    if (points.size() >= 2 && points.front().x > points.back().x) {
        std::reverse(points.begin(), points.end());
    }
}

// 侧向标定：贴线点标 ground 种子；无种子时侧向区域整段标负障碍物；返回种子数
int ScanTrendSegmenter::labelInitialSideRegionObstacles(
    std::vector<ScanPoint>& points, const LidarGroundCalib& calib) const
{
    if (!calib.side_blind_spot.loaded) {
        return 0;
    }

    const auto& region = calib.side_blind_spot;
    const float tol = params_.blind_spot_line_dist_tol;
    int ground_seed_count = 0;

    for (auto& point : points) {
        if (!isInSideSegmentXRange(point, region, params_.blind_spot_side_x_margin)) {
            continue;
        }
        float dist = 0.f;
        if (!distanceToSideLineSegment(point, region, dist)) {
            continue;
        }
        if (dist <= tol) {
            markGroundPoint(point);
            ++ground_seed_count;
        }
    }

    if (ground_seed_count > 0) {
        return ground_seed_count;
    }

    for (auto& point : points) {
        if (!isInSideSegmentXRange(point, region, params_.blind_spot_side_x_margin)) {
            continue;
        }
        float dist = 0.f;
        if (!distanceToSideLineSegment(point, region, dist)) {
            continue;
        }
        markNegativeObstacle(point);
    }
    return 0;
}

// 从 ground 种子沿 scan 双向传播：全局 max 向前、min 向后各一次
void ScanTrendSegmenter::propagateGroundAndDetectJumps(
    std::vector<ScanPoint>& points, const LidarGroundCalib& calib) const
{
    if (!calib.side_blind_spot.loaded || points.size() < 2) {
        return;
    }

    std::vector<std::pair<size_t, size_t>> seed_segments;
    collectSideGroundSeedSegments(points, calib.side_blind_spot, params_, seed_segments);

    size_t min_seed_idx = 0;
    size_t max_seed_idx = 0;
    if (!findSideGroundSeedIndexRange(seed_segments, min_seed_idx, max_seed_idx)) {
        return;
    }

    propagateGroundForward(points, max_seed_idx, params_);
    propagateGroundBackward(points, min_seed_idx, params_);
}

// x 条带 pos_slab：以已传播 ground 的 z 均值为 ref，检测凸/凹障碍
void ScanTrendSegmenter::detectPositiveObstacles(std::vector<ScanPoint>& points) const
{
    for (size_t i = 0; i < points.size(); ++i) {
        if (points[i].label == PointLabel::Obstacle) {
            continue;
        }

        float min_z = points[i].z;
        float max_z = points[i].z;
        float x_at_min_z = points[i].x;
        float x_at_max_z = points[i].x;
        std::vector<float> ground_zs;
        ground_zs.reserve(16);

        for (size_t j = 0; j < points.size(); ++j) {
            if (!inXGrid(points[i].x, points[j].x, params_.grid_x_max)) {
                continue;
            }
            if (points[j].label == PointLabel::Ground) {
                ground_zs.push_back(points[j].z);
            }
            if (points[j].z < min_z) {
                min_z = points[j].z;
                x_at_min_z = points[j].x;
            }
            if (points[j].z > max_z) {
                max_z = points[j].z;
                x_at_max_z = points[j].x;
            }
        }

        const float dz = max_z - min_z;
        if (dz <= params_.positive_dz_min) {
            continue;
        }

        const float dx = std::max(std::abs(x_at_max_z - x_at_min_z), params_.grid_x_min);
        if (dz / dx <= params_.positive_slope_min) {
            continue;
        }

        if (ground_zs.empty()) {
            continue;
        }

        float ref = std::abs(min_z) < std::abs(max_z) ? min_z : max_z;
        if (points[i].z >= ref + params_.positive_dz_min) {
            points[i].label = PointLabel::Obstacle;
            points[i].negative_obstacle = false;   // 凸起
        } else if (points[i].z <= ref - params_.positive_dz_min) {
            points[i].label = PointLabel::Obstacle;
            points[i].negative_obstacle = true;    // 下沉
        }
    }
}

// 将 |z| 很小的正障碍物还原为地面（当前 pipeline 默认未启用）
void ScanTrendSegmenter::revertNearGroundObstacles(std::vector<ScanPoint>& points) const
{
    for (auto& point : points) {
        if (point.label != PointLabel::Obstacle || point.negative_obstacle) {
            continue;
        }
        if (std::abs(point.z) < params_.positive_dz_min) {
            point.label = PointLabel::Ground;
        }
    }
}

// 镂空/格栅滤除：x 前后均有足够地面邻居的负障碍物改回 Ground
void ScanTrendSegmenter::filterGrateObstacles(std::vector<ScanPoint>& points) const
{
    for (size_t i = 0; i < points.size(); ++i) {
        if (!points[i].negative_obstacle || points[i].label != PointLabel::Obstacle) {
            continue;
        }

        // if (points[i].label != PointLabel::Obstacle) {
        //     continue;
        // }

        const float x_i = points[i].x;
        int back_neighbors = 0;
        int fwd_neighbors = 0;
        for (size_t j = 0; j < points.size(); ++j) {
            if (i == j || !isGrateNeighborCandidate(points[j], params_)) {
                continue;
            }
            const float dx = points[j].x - x_i;
            if (dx < 0.f) {
                if (-dx <= params_.grate_x_radius) {
                    ++back_neighbors;
                }
            } else if (dx > 0.f) {
                if (dx <= params_.grate_x_radius) {
                    ++fwd_neighbors;
                }
            }
        }

        if (back_neighbors >= params_.min_grate_back_neighbors &&
            fwd_neighbors >= params_.min_grate_fwd_neighbors) {
            points[i].label = PointLabel::Ground;
            points[i].negative_obstacle = false;
        }
    }
}

// 按 label 拆分 obstacle / ground / 全量 transform 点云
ScanTrendResult ScanTrendSegmenter::buildOutput(const std::vector<ScanPoint>& points) const
{
    ScanTrendResult result;
    result.obstacle = std::make_shared<PointCloud>();
    result.ground = std::make_shared<PointCloud>();
    result.transformed = std::make_shared<PointCloud>();

    for (const auto& point : points) {
        const PointT pcl_point(point.x, point.y, point.z);
        result.transformed->points.push_back(pcl_point);
        if (point.label == PointLabel::Obstacle) {
            result.obstacle->points.push_back(pcl_point);
        } else {
            result.ground->points.push_back(pcl_point);
        }
    }

    for (auto* cloud : {result.obstacle.get(), result.ground.get(), result.transformed.get()}) {
        cloud->width = cloud->points.size();
        cloud->height = 1;
        cloud->is_dense = false;
    }
    return result;
}

// blind spot 虚拟障碍：侧向 neg 足够且 ground 不足时，在 x≈0 注入虚拟点
void ScanTrendSegmenter::injectBlindSpotSideObstacles(
    const std::vector<ScanPoint>& points_after_grate,
    int neg_count_in_region_before_grate,
    const LidarGroundCalib& calib,
    ScanTrendResult& result) const
{
    if (!calib.side_blind_spot.loaded) {
        return;
    }
    if (neg_count_in_region_before_grate < params_.blind_spot_min_neg_pts) {
        return;
    }

    const auto& region = calib.side_blind_spot;
    int ground_count = 0;
    std::vector<float> ground_ys;
    ground_ys.reserve(points_after_grate.size());

    for (const auto& point : points_after_grate) {
        if (!isInSideBlindSpotRegion(point, region, params_.blind_spot_line_dist_tol)) {
            continue;
        }
        if (point.label != PointLabel::Ground) {
            continue;
        }
        if (std::abs(point.z) > params_.blind_spot_ground_z_tol) {
            continue;
        }
        ++ground_count;
        ground_ys.push_back(point.y);
    }

    if (ground_count >= params_.blind_spot_min_ground_pts) {
        return;
    }

    float virtual_y = 0.f;
    if (!ground_ys.empty()) {
        virtual_y = medianValue(ground_ys) * params_.blind_spot_y_scale;
    } else {
        virtual_y = region.start_y * 0.5f;
    }
    virtual_y = clampBlindSpotVirtualY(
        virtual_y, region.start_y, params_.blind_spot_virtual_y_min);

    for (float x = params_.blind_spot_virtual_x_min;
         x <= params_.blind_spot_virtual_x_max + 1e-4f;
         x += params_.blind_spot_virtual_x_step) {
        result.obstacle->points.emplace_back(x, virtual_y, 0.f);
    }
    result.obstacle->width = result.obstacle->points.size();
}

// 主流程：排序 → 初始 ground → 传播/jump → pos_slab → grate → 输出 → 虚拟障碍
ScanTrendResult ScanTrendSegmenter::process(
    const sensor_msgs::msg::LaserScan& scan,
    const LidarGroundCalib& calib) const
{
    const auto total_start = SteadyClock::now();
    ScanTrendTiming timing;
    ScanTrendResult empty;
    empty.obstacle = std::make_shared<PointCloud>();
    empty.ground = std::make_shared<PointCloud>();
    empty.transformed = std::make_shared<PointCloud>();

    if (!calib.loaded) {
        return empty;
    }

    auto step_start = SteadyClock::now();
    auto points = transformScan(scan, calib);
    timing.transform_us = elapsedUs(step_start);
    if (points.empty()) {
        timing.total_us = elapsedUs(total_start);
        empty.timing = timing;
        return empty;
    }

    step_start = SteadyClock::now();
    sortScanPoints(points);
    timing.sort_us = elapsedUs(step_start);

    step_start = SteadyClock::now();
    const int ground_seed_count = labelInitialSideRegionObstacles(points, calib);
    timing.label_initial_us = elapsedUs(step_start);

    int neg_count_in_region = 0;
    timing.run_ground_pipeline =
        !calib.side_blind_spot.loaded || ground_seed_count > 0;
    if (timing.run_ground_pipeline) {
        step_start = SteadyClock::now();
        propagateGroundAndDetectJumps(points, calib);
        timing.propagate_us = elapsedUs(step_start);

        step_start = SteadyClock::now();
        detectPositiveObstacles(points);
        timing.pos_slab_us = elapsedUs(step_start);

        step_start = SteadyClock::now();
        neg_count_in_region =
            countNegInSideRegion(points, calib.side_blind_spot, params_);
        timing.count_neg_us = elapsedUs(step_start);

        // revertNearGroundObstacles(points);
        step_start = SteadyClock::now();
        filterGrateObstacles(points);
        timing.grate_us = elapsedUs(step_start);
    } else {
        step_start = SteadyClock::now();
        neg_count_in_region =
            countNegInSideRegion(points, calib.side_blind_spot, params_);
        timing.count_neg_us = elapsedUs(step_start);
    }

    step_start = SteadyClock::now();
    auto result = buildOutput(points);
    timing.build_output_us = elapsedUs(step_start);

    step_start = SteadyClock::now();
    injectBlindSpotSideObstacles(points, neg_count_in_region, calib, result);
    timing.inject_virtual_us = elapsedUs(step_start);

    timing.n_points = points.size();
    timing.total_us = elapsedUs(total_start);
    result.timing = timing;
    return result;
}

// ROS 节点：加载标定、订阅左右 scan、发布 obstacle/debug 点云
LidarObstacleScanTrend::LidarObstacleScanTrend(const std::string& node_name)
    : Node(node_name)
{
    this->declare_parameter<std::string>(
        "calibration_dir",
        "/opt/freego/calibration_files");
    this->declare_parameter<std::string>("left_extrinsic", "left_tilt_lidar.txt");
    this->declare_parameter<std::string>("left_ground_yaml", "left_lidar_groung_model.yaml");
    this->declare_parameter<std::string>("right_extrinsic", "right_tilt_lidar.txt");
    this->declare_parameter<std::string>("right_ground_yaml", "right_lidar_ground_model.yaml");
    this->declare_parameter<bool>("timing_log_enable", false);
    this->declare_parameter<double>("timing_log_interval_sec", 5.0);
    this->declare_parameter<bool>("horiz_dedup_enable", true);
    this->declare_parameter<std::string>("dedup_target_frame", "base_link");
    this->declare_parameter<double>("dedup_radius_xy", 0.20);
    this->declare_parameter<double>("horiz_min_x", 0.0);

    timing_log_enable_ = this->get_parameter("timing_log_enable").as_bool();
    timing_log_interval_sec_ = this->get_parameter("timing_log_interval_sec").as_double();
    horiz_dedup_enable_ = this->get_parameter("horiz_dedup_enable").as_bool();
    dedup_target_frame_ = this->get_parameter("dedup_target_frame").as_string();
    dedup_radius_xy_ = static_cast<float>(this->get_parameter("dedup_radius_xy").as_double());
    horiz_min_x_ = static_cast<float>(this->get_parameter("horiz_min_x").as_double());

    horiz_left_base_ = std::make_shared<PointCloud>();
    horiz_right_base_ = std::make_shared<PointCloud>();
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const std::string calib_dir = this->get_parameter("calibration_dir").as_string();
    const std::string left_extrinsic = calib_dir + "/" + this->get_parameter("left_extrinsic").as_string();
    const std::string left_yaml = calib_dir + "/" + this->get_parameter("left_ground_yaml").as_string();
    const std::string right_extrinsic = calib_dir + "/" + this->get_parameter("right_extrinsic").as_string();
    const std::string right_yaml = calib_dir + "/" + this->get_parameter("right_ground_yaml").as_string();

    calib_left_.frame_id = "left_tilt_rotate";
    calib_right_.frame_id = "right_tilt_rotate";

    if (!loadExtrinsicTxt(left_extrinsic, calib_left_) || !loadGroundYaml(left_yaml, calib_left_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load left lidar calibration");
    } else {
        finalizeCalib(calib_left_);
        RCLCPP_INFO(this->get_logger(), "Loaded left calibration: tz=%.3f", calib_left_.tz);
    }

    if (!loadExtrinsicTxt(right_extrinsic, calib_right_) || !loadGroundYaml(right_yaml, calib_right_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load right lidar calibration");
    } else {
        finalizeCalib(calib_right_);
        RCLCPP_INFO(this->get_logger(), "Loaded right calibration: tz=%.3f", calib_right_.tz);
    }

    obstacle_pub_left_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/perception/left_tilt/pointcloud", 1);
    obstacle_pub_right_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
       "/perception/right_tilt/pointcloud", 1);
    debug_pub_left_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/transform_pointcloud2_left", 1);
    debug_pub_right_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/transform_pointcloud2_right", 1);

    scan_sub_left_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        leftTiltLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&LidarObstacleScanTrend::scanCallbackLeft, this, std::placeholders::_1));
    scan_sub_right_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        rightTiltLaserTopic, rclcpp::SensorDataQoS(),
        std::bind(&LidarObstacleScanTrend::scanCallbackRight, this, std::placeholders::_1));

    if (horiz_dedup_enable_) {
        horiz_sub_left_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            leftLaserTopic, rclcpp::SensorDataQoS(),
            std::bind(&LidarObstacleScanTrend::horizScanCallbackLeft, this, std::placeholders::_1));
        horiz_sub_right_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            rightLaserTopic, rclcpp::SensorDataQoS(),
            std::bind(&LidarObstacleScanTrend::horizScanCallbackRight, this, std::placeholders::_1));
        RCLCPP_INFO(
            this->get_logger(),
            "Horizontal dedup enabled: frame=%s radius=%.2f min_x=%.2f",
            dedup_target_frame_.c_str(),
            dedup_radius_xy_,
            horiz_min_x_);
    }
}

// 处理单帧 scan：分割并发布 obstacle 与 debug 点云
void LidarObstacleScanTrend::processScan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
    const LidarGroundCalib& calib,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub)
{
    if (!calib.loaded) {
        return;
    }

    const auto callback_start = SteadyClock::now();
    const ScanTrendResult result = segmenter_.process(*scan_msg, calib);

    auto step_start = SteadyClock::now();
    sensor_msgs::msg::PointCloud2 debug_msg;
    PCL2ROSMSg(result.transformed, debug_msg);
    const int64_t pcl_debug_us = elapsedUs(step_start);
    debug_msg.header = scan_msg->header;
    debug_msg.header.frame_id = calib.frame_id;

    step_start = SteadyClock::now();
    debug_pub->publish(debug_msg);
    const int64_t publish_debug_us = elapsedUs(step_start);

    PointCloud::Ptr obstacle_for_publish = result.obstacle;
    if (horiz_dedup_enable_) {
        obstacle_for_publish = dedupTiltObstaclesWithHorizontal(
            result.obstacle,
            calib.frame_id,
            rclcpp::Time(scan_msg->header.stamp),
            nullptr);
    }

    step_start = SteadyClock::now();
    sensor_msgs::msg::PointCloud2 obstacle_msg;
    PCL2ROSMSg(obstacle_for_publish, obstacle_msg);
    const int64_t pcl_obstacle_us = elapsedUs(step_start);
    obstacle_msg.header = scan_msg->header;
    obstacle_msg.header.frame_id = calib.frame_id;

    step_start = SteadyClock::now();
    obstacle_pub->publish(obstacle_msg);
    const int64_t publish_obstacle_us = elapsedUs(step_start);

    const int64_t callback_total_us = elapsedUs(callback_start);
    if (timing_log_enable_) {
        logCallbackTiming(
            this->get_logger(),
            calib.frame_id,
            callback_total_us,
            result.timing.total_us,
            pcl_debug_us,
            pcl_obstacle_us,
            publish_debug_us,
            publish_obstacle_us);
        logScanTrendTiming(this->get_logger(), calib.frame_id, result.timing);
    }
}

// 左 tilt 雷达 scan 回调
void LidarObstacleScanTrend::scanCallbackLeft(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    processScan(scan_msg, calib_left_, obstacle_pub_left_, debug_pub_left_);
}

// 右 tilt 雷达 scan 回调
void LidarObstacleScanTrend::scanCallbackRight(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    processScan(scan_msg, calib_right_, obstacle_pub_right_, debug_pub_right_);
}

void LidarObstacleScanTrend::horizScanCallbackLeft(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    updateHorizSideCloud(scan_msg, true);
}

void LidarObstacleScanTrend::horizScanCallbackRight(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    updateHorizSideCloud(scan_msg, false);
}

void LidarObstacleScanTrend::updateHorizSideCloud(
    const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, bool is_left)
{
    bool tf_ok = false;
    const PointCloud::Ptr cloud = horizScanToBaseLinkCloud(
        *scan_msg, *tf_buffer_, dedup_target_frame_, horiz_min_x_, &tf_ok);
    if (!tf_ok) {
        return;
    }

    std::lock_guard<std::mutex> lock(horiz_mutex_);
    if (is_left) {
        horiz_left_base_ = cloud;
    } else {
        horiz_right_base_ = cloud;
    }
}

PointCloud::Ptr LidarObstacleScanTrend::getHorizAllBaseLinkCloud() const
{
    auto merged = std::make_shared<PointCloud>();
    std::lock_guard<std::mutex> lock(horiz_mutex_);

    size_t total_points = 0;
    if (horiz_left_base_ != nullptr) {
        total_points += horiz_left_base_->points.size();
    }
    if (horiz_right_base_ != nullptr) {
        total_points += horiz_right_base_->points.size();
    }
    merged->points.reserve(total_points);

    if (horiz_left_base_ != nullptr) {
        merged->points.insert(
            merged->points.end(),
            horiz_left_base_->points.begin(),
            horiz_left_base_->points.end());
    }
    if (horiz_right_base_ != nullptr) {
        merged->points.insert(
            merged->points.end(),
            horiz_right_base_->points.begin(),
            horiz_right_base_->points.end());
    }

    merged->width = merged->points.size();
    merged->height = 1;
    merged->is_dense = false;
    return merged;
}

PointCloud::Ptr LidarObstacleScanTrend::dedupTiltObstaclesWithHorizontal(
    const PointCloud::ConstPtr& tilt_obstacle,
    const std::string& tilt_frame_id,
    const rclcpp::Time& stamp,
    bool* dedup_applied) const
{
    if (dedup_applied != nullptr) {
        *dedup_applied = false;
    }

    if (tilt_obstacle == nullptr) {
        return std::make_shared<PointCloud>();
    }
    if (tilt_obstacle->empty()) {
        if (dedup_applied != nullptr) {
            *dedup_applied = true;
        }
        return std::make_shared<PointCloud>(*tilt_obstacle);
    }

    const PointCloud::Ptr horiz_all = getHorizAllBaseLinkCloud();
    Eigen::Matrix4d tilt_to_base = Eigen::Matrix4d::Identity();
    const bool horiz_dedup_ready = !horiz_all->empty() && lookupTransformMatrix(
        *tf_buffer_, dedup_target_frame_, tilt_frame_id, stamp, tilt_to_base);

    auto filtered = std::make_shared<PointCloud>();
    filtered->points.reserve(tilt_obstacle->points.size());
    const float radius_sq = dedup_radius_xy_ * dedup_radius_xy_;

    for (const auto& point_tilt : tilt_obstacle->points) {
        if (point_tilt.z > kTiltMaxObstacleZ) {
            continue;
        }

        if (!horiz_dedup_ready) {
            filtered->points.push_back(point_tilt);
            continue;
        }

        const PointT point_base = transformPoint(point_tilt, tilt_to_base);
        bool matched = false;
        for (const auto& point_horiz : horiz_all->points) {
            const float dx = point_base.x - point_horiz.x;
            const float dy = point_base.y - point_horiz.y;
            if (dx * dx + dy * dy <= radius_sq) {
                matched = true;
                break;
            }
        }
        if (!matched) {
            filtered->points.push_back(point_tilt);
        }
    }

    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;

    if (dedup_applied != nullptr) {
        *dedup_applied = horiz_dedup_ready;
    }
    return filtered;
}
