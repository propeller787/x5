/**
 * @file vehicle_footprint.h
 * @brief 车体 footprint 与安全膨胀（凸多边形，4 或 6 顶点）
 *
 * 在 base_link 下定义 CCW 顶点轮廓，供 SweptFootprintChecker 扫掠碰撞。
 * 后轴为参考原点；支持单梯形（4 点）与双梯形（6 点）物理框。
 */
#pragma once

#include "safety_node/core/avoidance_types.h"

#include <utility>
#include <vector>

/** 轴对齐矩形边界（仅用于可视化外接框或兼容测试） */
struct FootprintRect
{
  double x_min = 0.0;
  double x_max = 0.0;
  double y_min = 0.0;
  double y_max = 0.0;
};

/** CCW 凸多边形：4 点单梯形，或 6 点双梯形（后左→分节左→前左→前右→分节右→后右） */
struct FootprintPolygon
{
  std::vector<std::pair<double, double>> vertices;
};

class VehicleFootprint {
public:
  explicit VehicleFootprint(const AvoidanceParams & params);

  void updateParams(const AvoidanceParams & params);

  /** 物理轮廓，不含 margin */
  FootprintPolygon physicalPolygon() const;

  /**
   * 物理车体在纵向 x 处的半宽（m）。
   * 双梯形时车头前段随两条斜边收窄；x 落在车体前/后端之外时按最近端半宽平推，
   * 便于自车点过滤沿真实轮廓（含前段斜边）收窄，避免斜边外障碍被误当自车滤除。
   */
  double physicalHalfWidthAtX(double x) const;

  /** 含 front/back/side margin 的膨胀轮廓 */
  FootprintPolygon inflatedPolygon() const;

  /** 物理轮廓外接矩形 */
  FootprintRect physicalRect() const;

  /** 膨胀轮廓外接矩形 */
  FootprintRect inflatedRect() const;

  /** 圆是否与凸多边形相交（扫掠碰撞） */
  static bool circleIntersectsPolygon(
    double x, double y, double obstacle_radius,
    const FootprintPolygon & polygon);

  /** 圆是否完全落在凸多边形内（含边界，用于 margin 判定） */
  static bool circleContainedInPolygon(
    double x, double y, double obstacle_radius,
    const FootprintPolygon & polygon);

  /** 圆 (x,y,r) 是否与凸多边形相交 */
  static bool pointInPolygon(
    double x, double y, double obstacle_radius,
    const FootprintPolygon & polygon);

  /** 圆到多边形边界净空（负值表示侵入/重叠） */
  static double clearanceToPolygon(
    double x, double y, double obstacle_radius,
    const FootprintPolygon & polygon);

  /**
   * 符号净空：圆心在多边形外为正（边界距离 − 半径），在内为负（−穿深 − 半径）。
   * 与 clearanceToPolygon 不同，跨越边界单调递减，可用于「运动是否使障碍变得更危险」
   * 的趋近性判定（方向掩码 / 近障方向豁免）。
   */
  static double signedClearanceToPolygon(
    double x, double y, double obstacle_radius,
    const FootprintPolygon & polygon);

  /** 将 base_link 多边形变换到轨迹 pose 坐标系 */
  static FootprintPolygon transformPolygon(
    const FootprintPolygon & polygon,
    const PredictedPose & pose);

  /** 多边形内部与边界采样点（map 查询用，pose 坐标系） */
  static std::vector<std::pair<double, double>> samplePolygonInterior(
    const FootprintPolygon & polygon,
    double step_m);

  static bool pointInRect(double x, double y, double obstacle_radius, const FootprintRect & rect);
  static double clearanceToRect(
    double x, double y, double obstacle_radius,
    const FootprintRect & rect);

private:
  static FootprintPolygon buildPolygon(
    double x_rear,
    double x_front,
    double width_rear,
    double width_front);

  static FootprintPolygon buildDualTrapezoidPolygon(
    double x_rear,
    const FootprintSegmentParams & rear_segment,
    const FootprintSegmentParams & front_segment);

  static FootprintPolygon inflateAxisAlignedTrapezoid(
    double x_rear,
    double x_front,
    double width_rear,
    double width_front,
    double front_margin,
    double back_margin,
    double side_margin);

  static FootprintPolygon offsetPolygonByEdgeMargins(
    const FootprintPolygon & polygon,
    const std::vector<double> & edge_margins_m);

  static FootprintRect boundingRect(const FootprintPolygon & polygon);

  AvoidanceParams params_;
};
