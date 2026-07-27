# shared_control_assist

Single-node MVP for the shared-control assist plan in `src/scene/方案/方案.md`.

## Node

Executable:

```bash
ros2 launch shared_control_assist shared_control_assist.launch.py
```

Main node:

```text
shared_control_assist
```

## Inputs

```text
/handle/control   geometry_msgs/msg/Twist
/wheel/twist      geometry_msgs/msg/TwistStamped
/scan/left        sensor_msgs/msg/LaserScan
/scan/right       sensor_msgs/msg/LaserScan
/scan/back        sensor_msgs/msg/LaserScan
/assist/enable    std_msgs/msg/Bool
```

## Outputs

```text
/nav/control                 geometry_msgs/msg/Twist
/assist/points               sensor_msgs/msg/PointCloud2
/assist/local_grid           nav_msgs/msg/OccupancyGrid
/assist/state                std_msgs/msg/String
/assist/debug/markers        visualization_msgs/msg/MarkerArray
/assist/debug/reason         std_msgs/msg/String
/assist/debug/door_score     std_msgs/msg/Float32
```

## MVP Scope

- Fuses three horizontal `LaserScan` streams into `base_link` through TF.
- Computes local occupancy grid, obstacle distances, slow zones, and software stop conditions.
- Detects narrow-door candidates with a max-free-gap method and a parallel-wall fallback.
- Runs a narrow-door state machine and blends joystick intent with angular assist.
- Publishes the final safe command directly to `/nav/control`.

This package intentionally avoids dependencies on existing scene nodes so it can be tested and replaced independently.
