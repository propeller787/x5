#!/usr/bin/env python3
# =============================================================================
# fusion_validator.py —— lidar_imu_fusion 输出数据稳定性与准确性验证工具
#
# 订阅 /perception/lidar_fusion/scan，实时采集并统计以下指标，结束时输出报告：
#   1. 帧率           接收间隔的均值/方差/P99，理论应≈10 Hz
#   2. 端到端延迟     now() - header.stamp 的均值/方差/P99，应 < 100 ms
#   3. stamp 单调性   相邻帧 stamp 是否单调递增，统计逆序次数
#   4. 空 bin 占比    每帧 inf/NaN 比例的均值（空旷场景应高，cluttered 应低）
#   5. 有效点数       每帧有限 ranges 数量的均值/方差
#   6. 静态障碍稳定性 指定角度 bin 的距离序列均值/方差/极差/P99 波动
#                     （机器人静止时该指标衡量噪声；运动时衡量去畸变残留拖影）
#   7. 角度覆盖率     有障碍的 bin 数 / 总 bin 数
#
# 用法:
#   # 方式 A: 直接 python3 运行（需先 source ROS 环境 + 融合节点已在跑）
#   python3 scripts/fusion_validator.py --duration 30 --watch-angle 0.0
#
#   # 方式 B: ros2 run（需 colcon build 后 source install/setup.bash）
#   ros2 run lidar_imu_fusion fusion_validator.py --duration 30 --watch-angle 0.0
#
#   # --duration 0 表示持续运行直到 Ctrl+C
#   # --watch-angle 指定要监控稳定性的角度（度，0=正前方），需对该方向放静止障碍
#   # --output report.json 把最终报告写入文件
# =============================================================================

import argparse
import json
import math
import statistics
import sys
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def percentile(sorted_values, p):
    """计算已排序序列的 P 分位数 (p in [0, 100])."""
    if not sorted_values:
        return float("nan")
    if len(sorted_values) == 1:
        return sorted_values[0]
    k = (len(sorted_values) - 1) * (p / 100.0)
    f = math.floor(k)
    c = math.ceil(k)
    if f == c:
        return sorted_values[int(k)]
    return sorted_values[f] + (sorted_values[c] - sorted_values[f]) * (k - f)


def safe_mean(values):
    return statistics.fmean(values) if values else float("nan")


def safe_variance(values):
    if len(values) < 2:
        return float("nan")
    return statistics.pvariance(values)  # 总体方差，整段统计用


class FusionValidator(Node):
    def __init__(self, args):
        super().__init__("fusion_validator")
        self.duration = args.duration
        self.watch_angle_deg = args.watch_angle
        self.output_path = args.output

        # —— 采集容器 ——
        self._recv_monotonic = deque()       # 每帧接收的 monotonic 时刻（帧率统计）
        self._latencies_ms = deque()         # 端到端延迟（ms）
        self._stamps_ns = deque()            # header.stamp 纳秒序列（单调性）
        self._mono_violations = 0            # stamp 非单调次数
        self._empty_ratios = deque()         # 每帧空 bin 占比
        self._valid_counts = deque()         # 每帧有效点数
        self._coverage_ratios = deque()      # 每帧有障碍 bin 占比
        self._watch_ranges = deque()         # 监控角度 bin 的距离序列
        self._watch_invalid = 0              # 监控 bin 无效次数
        self._watch_index = None             # 监控 bin 索引（首帧确定）

        self._lock = threading.Lock()
        self._frame_count = 0

        # 必须与融合节点发布端 QoS 一致（qos_profile_sensor_data = BEST_EFFORT, KEEP_LAST），
        # 否则会出现 "incompatible QoS, no messages will be received" 警告且收不到任何数据。
        self._sub = self.create_subscription(
            LaserScan, "/perception/lidar_fusion/scan", self._on_scan, qos_profile_sensor_data
        )

        # 定时打印实时摘要
        self._report_timer = self.create_timer(2.0, self._print_live_summary)

        # duration > 0 时到点自动停止
        if self.duration and self.duration > 0:
            self._stop_timer = self.create_timer(
                float(self.duration), self._stop_and_report
            )
        else:
            self._stop_timer = None

        self.get_logger().info(
            f"fusion_validator 启动: 监听 /perception/lidar_fusion/scan, "
            f"duration={'∞' if not self.duration else self.duration}s, "
            f"watch_angle={self.watch_angle_deg}°"
        )

    def _on_scan(self, msg: LaserScan):
        with self._lock:
            self._frame_count += 1
            now_monotonic = self.get_clock().now().nanoseconds  # ROS 时钟
            self._recv_monotonic.append(now_monotonic)

            # 端到端延迟: now - header.stamp
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if stamp_ns > 0:
                latency_ns = now_monotonic - stamp_ns
                if latency_ns >= 0:
                    self._latencies_ms.append(latency_ns / 1_000_000.0)

            # stamp 单调性
            if self._stamps_ns and stamp_ns < self._stamps_ns[-1]:
                self._mono_violations += 1
            self._stamps_ns.append(stamp_ns)

            # 空 bin / 有效点 / 覆盖率
            total = len(msg.ranges)
            if total > 0:
                invalid = sum(
                    1 for r in msg.ranges
                    if math.isinf(r) or math.isnan(r)
                )
                valid = total - invalid
                self._empty_ratios.append(invalid / total)
                self._valid_counts.append(valid)
                # 覆盖率: 非空且距离在合理范围的 bin
                covered = sum(
                    1 for r in msg.ranges
                    if math.isfinite(r) and 0.0 < r < msg.range_max
                )
                self._coverage_ratios.append(covered / total)

                # 监控角度 bin（首帧确定索引）
                if self._watch_index is None and msg.angle_increment > 0:
                    watch_rad = math.radians(self.watch_angle_deg)
                    raw = (watch_rad - msg.angle_min) / msg.angle_increment
                    self._watch_index = int(round(raw))
                    if self._watch_index < 0 or self._watch_index >= total:
                        self._watch_index = None
                        self.get_logger().warn(
                            f"watch_angle {self.watch_angle_deg}° 超出输出角度范围 "
                            f"[{math.degrees(msg.angle_min):.1f}, "
                            f"{math.degrees(msg.angle_max):.1f}]，将跳过稳定性统计"
                        )

                if self._watch_index is not None:
                    r = msg.ranges[self._watch_index]
                    if math.isfinite(r) and r > 0.0:
                        self._watch_ranges.append(float(r))
                    else:
                        self._watch_invalid += 1

    def _print_live_summary(self):
        with self._lock:
            n = self._frame_count
            if n < 2:
                return
            # 最近 50 帧的实时帧率
            recent = list(self._recv_monotonic)[-50:]
            if len(recent) >= 2:
                dt_ms = [(recent[i] - recent[i - 1]) / 1_000_000.0
                         for i in range(1, len(recent))]
                hz = 1000.0 / safe_mean(dt_ms) if safe_mean(dt_ms) > 0 else 0.0
            else:
                hz = 0.0
            lat = (safe_mean(list(self._latencies_ms)[-50:])
                   if self._latencies_ms else float("nan"))
            empty = (safe_mean(list(self._empty_ratios)[-50:])
                     if self._empty_ratios else float("nan"))
        self.get_logger().info(
            f"[实时] frames={n} hz≈{hz:.2f} lat≈{lat:.1f}ms empty≈{empty*100:.1f}%"
        )

    def _stop_and_report(self):
        self._report_timer.cancel()
        if self._stop_timer is not None:
            self._stop_timer.cancel()
        report = self._build_report()
        self._print_report(report)
        if self.output_path:
            with open(self.output_path, "w", encoding="utf-8") as f:
                json.dump(report, f, ensure_ascii=False, indent=2)
            self.get_logger().info(f"报告已写入 {self.output_path}")
        rclpy.shutdown()

    def _build_report(self):
        with self._lock:
            # 帧率: 基于接收间隔
            recv = list(self._recv_monotonic)
            intervals_ms = (
                [(recv[i] - recv[i - 1]) / 1_000_000.0
                 for i in range(1, len(recv))]
                if len(recv) >= 2 else []
            )
            if intervals_ms and safe_mean(intervals_ms) > 0:
                hz_mean = 1000.0 / safe_mean(intervals_ms)
            else:
                hz_mean = float("nan")

            lat = sorted(list(self._latencies_ms))
            empty = list(self._empty_ratios)
            valid = list(self._valid_counts)
            cov = list(self._coverage_ratios)
            watch = list(self._watch_ranges)

            sorted_intervals = sorted(intervals_ms)
            report = {
                "frame_count": self._frame_count,
                "frame_rate_hz": {
                    "mean": hz_mean,
                    "interval_ms_p50": (
                        percentile(sorted_intervals, 50)
                        if intervals_ms else float("nan")),
                    "interval_ms_p99": (
                        percentile(sorted_intervals, 99)
                        if intervals_ms else float("nan")),
                    "interval_ms_stddev": (
                        math.sqrt(safe_variance(intervals_ms))
                        if len(intervals_ms) >= 2 else float("nan")),
                },
                "latency_ms": {
                    "mean": safe_mean(lat),
                    "p50": percentile(lat, 50),
                    "p99": percentile(lat, 99),
                    "max": lat[-1] if lat else float("nan"),
                    "stddev": (
                        math.sqrt(safe_variance(lat))
                        if len(lat) >= 2 else float("nan")),
                },
                "stamp_monotonicity": {
                    "violations": self._mono_violations,
                    "sample_count": len(self._stamps_ns),
                },
                "empty_bin_ratio": {
                    "mean": safe_mean(empty),
                    "stddev": (
                        math.sqrt(safe_variance(empty))
                        if len(empty) >= 2 else float("nan")),
                },
                "valid_points_per_frame": {
                    "mean": safe_mean(valid),
                    "stddev": (
                        math.sqrt(safe_variance(valid))
                        if len(valid) >= 2 else float("nan")),
                },
                "coverage_ratio": {
                    "mean": safe_mean(cov),
                },
                "watch_angle_deg": self.watch_angle_deg,
                "watch_angle_stability": {
                    "samples": len(watch),
                    "invalid_samples": self._watch_invalid,
                    "range_mean_m": safe_mean(watch),
                    "range_stddev_m": (
                        math.sqrt(safe_variance(watch))
                        if len(watch) >= 2 else float("nan")),
                    "range_ptp_m": (
                        (max(watch) - min(watch))
                        if watch else float("nan")),
                    "range_p99_jitter_m": (
                        percentile(sorted(
                            [abs(watch[i] - watch[i - 1])
                             for i in range(1, len(watch))]),
                            99)
                        if len(watch) >= 2 else float("nan")),
                },
            }
            return report

    def _print_report(self, r):
        def fmtn(v, unit=""):
            if isinstance(v, float) and math.isnan(v):
                return "N/A"
            return f"{v:.4f}{unit}" if isinstance(v, float) else f"{v}{unit}"

        lines = []
        lines.append("=" * 70)
        lines.append("lidar_imu_fusion 输出验证报告")
        lines.append("=" * 70)
        lines.append(f"总帧数: {r['frame_count']}")
        lines.append("")
        lines.append("【帧率】(理论 ≈ 10 Hz)")
        lines.append(f"  均值           : {fmtn(r['frame_rate_hz']['mean'], ' Hz')}")
        lines.append(f"  间隔 P50       : {fmtn(r['frame_rate_hz']['interval_ms_p50'], ' ms')}")
        lines.append(f"  间隔 P99       : {fmtn(r['frame_rate_hz']['interval_ms_p99'], ' ms')}")
        lines.append(f"  间隔 标准差    : {fmtn(r['frame_rate_hz']['interval_ms_stddev'], ' ms')}")
        lines.append("")
        lines.append("【端到端延迟】(应 < 100 ms)")
        lines.append(f"  均值           : {fmtn(r['latency_ms']['mean'], ' ms')}")
        lines.append(f"  P50            : {fmtn(r['latency_ms']['p50'], ' ms')}")
        lines.append(f"  P99            : {fmtn(r['latency_ms']['p99'], ' ms')}")
        lines.append(f"  最大           : {fmtn(r['latency_ms']['max'], ' ms')}")
        lines.append(f"  标准差         : {fmtn(r['latency_ms']['stddev'], ' ms')}")
        lines.append("")
        lines.append("【stamp 单调性】(violations 应为 0)")
        lines.append(f"  逆序次数       : {r['stamp_monotonicity']['violations']}")
        lines.append(f"  样本数         : {r['stamp_monotonicity']['sample_count']}")
        lines.append("")
        lines.append("【空 bin 占比】(空旷场景应高; cluttered 应低且稳定)")
        lines.append(f"  均值           : {fmtn(r['empty_bin_ratio']['mean'] * 100, ' %')}")
        lines.append(f"  标准差         : {fmtn(r['empty_bin_ratio']['stddev'] * 100, ' %')}")
        lines.append("")
        lines.append("【有效点数/帧】")
        lines.append(f"  均值           : {fmtn(r['valid_points_per_frame']['mean'])}")
        lines.append(f"  标准差         : {fmtn(r['valid_points_per_frame']['stddev'])}")
        lines.append("")
        lines.append("【角度覆盖率】(有障碍 bin 占比)")
        lines.append(f"  均值           : {fmtn(r['coverage_ratio']['mean'] * 100, ' %')}")
        lines.append("")
        ws = r["watch_angle_stability"]
        lines.append(f"【监控角度 {r['watch_angle_deg']}° 稳定性】"
                     f"(机器人静止时应极小; 运动时衡量去畸变残留)")
        lines.append(f"  有效样本       : {ws['samples']}  (无效: {ws['invalid_samples']})")
        lines.append(f"  距离均值       : {fmtn(ws['range_mean_m'], ' m')}")
        lines.append(f"  距离标准差     : {fmtn(ws['range_stddev_m'], ' m')}")
        lines.append(f"  距离峰峰值     : {fmtn(ws['range_ptp_m'], ' m')}")
        lines.append(f"  帧间跳变 P99   : {fmtn(ws['range_p99_jitter_m'], ' m')}")
        lines.append("")
        lines.append("=" * 70)
        msg = "\n".join(lines)
        print(msg, file=sys.stderr)
        self.get_logger().info("验证完成")


def main():
    parser = argparse.ArgumentParser(
        description="lidar_imu_fusion 输出数据稳定性与准确性验证工具"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="采集时长(秒); 0=持续运行直到 Ctrl+C"
    )
    parser.add_argument(
        "--watch-angle", type=float, default=0.0,
        help="要监控稳定性的角度(度, 0=正前方); 需在该方向放静止障碍"
    )
    parser.add_argument(
        "--output", type=str, default="",
        help="把最终报告写入指定 JSON 文件路径"
    )
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = FusionValidator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C, 生成最终报告...")
        report = node._build_report()
        node._print_report(report)
        if node.output_path:
            with open(node.output_path, "w", encoding="utf-8") as f:
                json.dump(report, f, ensure_ascii=False, indent=2)
            node.get_logger().info(f"报告已写入 {node.output_path}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
