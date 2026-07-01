# lidar_imu_fusion

Standalone ROS 2 node for fusing the left, right, and back 2D lidars into one
deskewed `sensor_msgs/msg/LaserScan`.

## Inputs

- `/scan/left` (`sensor_msgs/msg/LaserScan`)
- `/scan/right` (`sensor_msgs/msg/LaserScan`)
- `/scan/back` (`sensor_msgs/msg/LaserScan`)
- `/sensor/imu/data` (`sensor_msgs/msg/Imu`)

## Output

- `/perception/lidar_fusion/scan` (`sensor_msgs/msg/LaserScan`)

The output scan is published in `target_frame`, which defaults to `base_link`.

## Processing

1. Cache the three 10 Hz lidar streams and select scan triplets whose header
   timestamps are within `max_sync_slop_sec`. Stale scans that can no longer
   form a valid triplet are pruned on every synchronization attempt.
2. Cache 100 Hz IMU angular velocity samples (transformed into `target_frame`).
3. Synchronized triplets are placed on a small pending queue and processed only
   once IMU coverage is available (when `require_imu_for_publish` is true), so a
   missing or lagging IMU never silently produces un-deskewed output.
4. Convert each valid lidar beam to a point.
5. Use `header.stamp + index * time_increment` as the per-beam timestamp.
6. Integrate IMU angular velocity from each beam timestamp to the selected
   fusion reference time.
7. Rotate each point into the reference time, transform it to `target_frame`,
   and rasterize the merged points back to a single `LaserScan`.

Only rotational deskew is applied. Translational deskew is intentionally not
included because this standalone module does not consume wheel odometry. TF
lookups use the latest available transform (`TimePointZero`), which assumes the
lidars and IMU are rigidly mounted on `target_frame`.

### Per-sensor fault tolerance

If the transform for one of the three lidars is unavailable, only that sensor
is skipped and the remaining two are still fused and published. The whole group
is dropped only when IMU coverage is required (`require_imu_for_publish`) and
is lost mid-scan, or when all three transforms are unavailable.

## Run

```bash
ros2 launch lidar_imu_fusion lidar_imu_fusion.launch.py
```

## Configuration

Edit `config/lidar_imu_fusion.yaml`.

### Defaults

The node defaults (and the YAML) are aligned so that running the node with or
without the launch file behaves consistently:

- `fallback_to_static_transforms: true`
- `require_imu_for_publish: false`
- `output_angle_increment: 0.004363323129985824` (~0.25 deg)

If `output_angle_increment` is set to a non-positive value it is reset to
`pi/720`; the output resolution is never silently derived from the input scans.

The node first tries TF for `target_frame <- laser_frame_*` and
`target_frame <- imu_link`. If TF is unavailable and
`fallback_to_static_transforms` is true, it uses the static transforms in the
YAML file. The IMU->target rotation is cached and refreshed from TF roughly
once per second rather than on every 100 Hz IMU sample.

## Diagnostics

- `LaserScan header.stamp is 0` / `IMU header.stamp is 0` (warned once): the
  driver is not filling the timestamp; the node falls back to `now()`. Fix the
  driver clock rather than relying on this fallback.
- `... kept un-deskewed due to missing IMU coverage`: beams were kept in the
  output but without rotational deskew because IMU data was missing for their
  time interval (only when `require_imu_for_publish` is false).
