/**
 * @file direction_mask.h
 * @brief L1 方向解耦掩码：近障按轴封锁 + 轨迹威胁归因谓词
 *
 * 核心思想：障碍进入硬停带（相对膨胀轮廓的符号净空 ≤ hard_stop_distance_m，
 * 含已在虚拟/物理车体内）后，不再对整条 Twist 做「安全/不安全」二值判定，
 * 而是用 ±v / ±w 四组几何短探针分别判定：该轴的运动是否将此近障推向物理接触
 * （符号净空跌破接触缓冲，且相对初值显著减小）。只有受威胁的轴被封锁。
 *
 * 采用探针而非扇区分类的原因：差速车原地旋转时车体前半与后半横移方向相反，
 * 左后角障碍实际被「右转」逼近而非「左转」——扇区规则会同时误封与漏封，
 * 探针沿真实扫掠几何求净空则天然正确处理四角与前后位置。
 *
 * 同一谓词 obstacleThreatenedAlongPoses 供三处复用：
 *   1) computeDirectionBlockMask：四轴探针 → 掩码 + 每轴威胁净空
 *   2) SweptFootprintChecker：候选轨迹的近障方向豁免
 *   3) AvoidanceController：LocalMap veto 的近障豁免圆收集
 */
#pragma once

#include "safety_node/core/avoidance_types.h"
#include "safety_node/core/vehicle_footprint.h"

#include <limits>
#include <vector>

/** 四向近障封锁掩码。left/right 对应 +w/−w（原地左转/右转）。 */
struct DirectionBlockMask
{
  bool forward = false;
  bool backward = false;
  bool left = false;
  bool right = false;
  /**
   * 各轴受威胁近障在 t=0 相对膨胀轮廓的最小符号净空 (m)；无威胁为 +inf。
   * 供 MDB 定向钳制：仅前向威胁净空参与前向制动距离压缩。
   */
  double forward_clearance_m = std::numeric_limits<double>::infinity();
  double backward_clearance_m = std::numeric_limits<double>::infinity();
  double left_clearance_m = std::numeric_limits<double>::infinity();
  double right_clearance_m = std::numeric_limits<double>::infinity();

  bool any() const
  {
    return forward || backward || left || right;
  }
};

namespace direction_mask
{

/** 近障资格：t=0 相对膨胀轮廓的符号净空 ≤ hard_stop_distance_m（含体内负值） */
bool isNearObstacle(
  const ObstaclePrimitive & obs,
  const FootprintPolygon & inflated_polygon,
  double hard_stop_distance_m);

/**
 * 障碍沿位姿序列是否被推向物理接触。
 *
 * 判据：min_{poses} 符号净空 < min(contact_buffer_m, 初始符号净空) − approach_eps_m。
 * 即：运动使障碍跌破接触缓冲；或障碍已在缓冲内/体内时穿深继续加大。
 * 远离、平行掠过、净空恒定的运动均不构成威胁。
 *
 * @param physical_polygon 物理车体轮廓（base_link）——威胁以真实接触度量，
 *        膨胀余量的管控仍由主避障链负责。
 */
bool obstacleThreatenedAlongPoses(
  const ObstaclePrimitive & obs,
  const std::vector<PredictedPose> & poses,
  const FootprintPolygon & physical_polygon,
  double contact_buffer_m,
  double approach_eps_m);

/** 生成纯平移短探针位姿序列（direction_sign=+1 前向 / −1 后向），首位姿为原点 */
std::vector<PredictedPose> linearProbePoses(
  double direction_sign,
  double distance_m,
  double step_m);

/** 生成原地旋转短探针位姿序列（direction_sign=+1 左转 / −1 右转），首位姿为原点 */
std::vector<PredictedPose> angularProbePoses(
  double direction_sign,
  double angle_rad,
  double step_rad);

/**
 * 对整场障碍计算四向封锁掩码。
 *
 * 遍历全部有效近障（不只 worst），每障碍对四组探针分别做威胁判定；
 * 任一近障威胁某轴即封锁该轴，并记录该轴受威胁近障的最小膨胀净空。
 */
DirectionBlockMask computeDirectionBlockMask(
  const ObstacleField & field,
  const FootprintPolygon & physical_polygon,
  const FootprintPolygon & inflated_polygon,
  const AvoidanceParams & params);

}  // namespace direction_mask
