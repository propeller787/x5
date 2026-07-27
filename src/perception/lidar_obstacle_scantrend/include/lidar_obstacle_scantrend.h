#ifndef LIDAR_OBSTACLE_SCANTREND_H
#define LIDAR_OBSTACLE_SCANTREND_H

#include "perception_utils.h"
#include "utils/topic.h"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

enum class PointLabel : uint8_t {
    Ground = 1,
    Obstacle = 2,
};

struct ScanPoint {
    float x = 0.f;
    float y = 0.f;
    float z = 0.f;
    float angle = 0.f;
    float range = 0.f;
    uint16_t beam_index = 0;
    PointLabel label = PointLabel::Ground;
    bool negative_obstacle = false;
};

struct BlindSpotSideRegion {
    float start_x = 0.f;
    float start_y = 0.f;
    float start_z = 0.f;
    float end_x = 0.f;
    float end_y = 0.f;
    float end_z = 0.f;
    bool loaded = false;
};

struct LidarGroundCalib {
    double tz = 0.0;
    double ty = 0.0;
    double tx = 0.0;
    double rz = 0.0;
    double ry = 0.0;
    double rx = 0.0;
    double polar_k = 0.0;
    double polar_b = 0.0;

    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    Eigen::Vector3d translation = Eigen::Vector3d::Zero();
    BlindSpotSideRegion side_blind_spot;
    std::string frame_id;
    bool loaded = false;
};

struct ScanTrendParams {
    float grid_x_min = 0.10f;
    float grid_x_max = 0.20f;
    float positive_dz_min = 0.13f;
    float positive_slope_min = 0.40f;
    float negative_dx_min = 1.0f;  // legacy; cliff edge uses max_expand_dx
    float negative_dz_min = 0.10f;
    float depression_low_z_thresh = -0.08f;
    float platform_high_z_thresh = 0.13f;
    float sink_z_tol = 0.06f;
    float ramp_step_dz = 0.06f;  // legacy; gradual ramp uses slope consistency
    float ramp_slope_max = 0.45f;
    float ramp_min_dx = 0.10f;
    float ramp_slope_abs_tol = 0.08f;
    float ramp_slope_ratio_tol = 0.35f;
    int ramp_min_valid_slopes = 2;
    float max_sink_x = 10.0f;
    float max_expand_dx = 1.50f;
    int z_ref_window = 6;
    int max_sink_points = 1000;
    int max_cliff_points = 120;
    float grate_x_radius = 0.20f;
    int min_grate_back_neighbors = 10;
    int min_grate_fwd_neighbors = 10;
    float noise_check_range = 0.25f;
    int noise_min_count = 10;
    float max_point_z = 1.5f;
    float blind_spot_line_dist_tol = 0.10f;
    float blind_spot_side_x_margin = 0.05f;
    float blind_spot_ground_z_tol = 0.10f;
    float blind_spot_neg_z_thresh = -0.08f;
    int blind_spot_min_ground_pts = 80;
    int blind_spot_min_neg_pts = 2;
    float blind_spot_virtual_y_min = 0.10f;
    float blind_spot_y_scale = 1.0f;
    float blind_spot_virtual_x_min = -0.10f;
    float blind_spot_virtual_x_max = 0.10f;
    float blind_spot_virtual_x_step = 0.05f;
};

struct ScanTrendTiming {
    int64_t total_us = 0;
    int64_t transform_us = 0;
    int64_t sort_us = 0;
    int64_t label_initial_us = 0;
    int64_t propagate_us = 0;
    int64_t pos_slab_us = 0;
    int64_t count_neg_us = 0;
    int64_t grate_us = 0;
    int64_t build_output_us = 0;
    int64_t inject_virtual_us = 0;
    size_t n_points = 0;
    bool run_ground_pipeline = false;
};

struct ScanTrendResult {
    PointCloud::Ptr obstacle;
    PointCloud::Ptr ground;
    PointCloud::Ptr transformed;
    ScanTrendTiming timing;
};

// scan-trend 分割器：基于标定 ground 传播 + jump + pos_slab 的障碍物检测
class ScanTrendSegmenter {
public:
    explicit ScanTrendSegmenter(ScanTrendParams params = {});

    // 处理一帧 LaserScan，返回 obstacle/ground/transform 点云
    ScanTrendResult process(
        const sensor_msgs::msg::LaserScan& scan,
        const LidarGroundCalib& calib) const;

private:
    ScanTrendParams params_;

    bool checkScanNoise(const sensor_msgs::msg::LaserScan& scan) const;
    std::vector<ScanPoint> transformScan(
        const sensor_msgs::msg::LaserScan& scan,
        const LidarGroundCalib& calib) const;
    void sortScanPoints(std::vector<ScanPoint>& points) const;
    int labelInitialSideRegionObstacles(
        std::vector<ScanPoint>& points, const LidarGroundCalib& calib) const;
    void propagateGroundAndDetectJumps(
        std::vector<ScanPoint>& points, const LidarGroundCalib& calib) const;
    void detectPositiveObstacles(std::vector<ScanPoint>& points) const;
    void revertNearGroundObstacles(std::vector<ScanPoint>& points) const;
    void filterGrateObstacles(std::vector<ScanPoint>& points) const;
    ScanTrendResult buildOutput(const std::vector<ScanPoint>& points) const;
    void injectBlindSpotSideObstacles(
        const std::vector<ScanPoint>& points_after_grate,
        int neg_count_in_region_before_grate,
        const LidarGroundCalib& calib,
        ScanTrendResult& result) const;
};

// 加载外参 txt
bool loadExtrinsicTxt(const std::string& path, LidarGroundCalib& calib);
// 加载地面模型 yaml
bool loadGroundYaml(const std::string& path, LidarGroundCalib& calib);
// 计算标定旋转/平移矩阵
void finalizeCalib(LidarGroundCalib& calib);

// ROS2 节点：左右 tilt 雷达 scan-trend 障碍物检测
class LidarObstacleScanTrend : public rclcpp::Node {
public:
    explicit LidarObstacleScanTrend(const std::string& node_name);

private:
    void scanCallbackLeft(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void scanCallbackRight(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void horizScanCallbackLeft(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void horizScanCallbackRight(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);
    void processScan(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg,
        const LidarGroundCalib& calib,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub,
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub);
    void updateHorizSideCloud(
        const sensor_msgs::msg::LaserScan::SharedPtr scan_msg, bool is_left);
    PointCloud::Ptr getHorizAllBaseLinkCloud() const;
    PointCloud::Ptr dedupTiltObstaclesWithHorizontal(
        const PointCloud::ConstPtr& tilt_obstacle,
        const std::string& tilt_frame_id,
        const rclcpp::Time& stamp,
        bool* dedup_applied) const;

    ScanTrendSegmenter segmenter_;
    LidarGroundCalib calib_left_;
    LidarGroundCalib calib_right_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_left_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_right_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr horiz_sub_left_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr horiz_sub_right_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_pub_right_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_right_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    mutable std::mutex horiz_mutex_;
    PointCloud::Ptr horiz_left_base_;
    PointCloud::Ptr horiz_right_base_;

    std::string dedup_target_frame_ = "base_link";
    float dedup_radius_xy_ = 0.20f;
    float horiz_min_x_ = 0.0f;
    bool horiz_dedup_enable_ = true;

    bool timing_log_enable_ = false;
    double timing_log_interval_sec_ = 5.0;
    int64_t last_timing_log_ns_ = 0;
};

#endif  // LIDAR_OBSTACLE_SCANTREND_H
