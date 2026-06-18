# 斜雷达近距障碍快照：可用于调试发布，也可作为 safety_node 可选输入（已处于 base_link）
std_msgs/Header header
string source_name
uint8 source_id
TiltObstaclePoint[] filtered_points
TiltObstaclePoint[] primitives
float32 nearest_forward_edge_m
float32 min_ttc_s
