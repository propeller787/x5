#pragma once

#include "avoidance_types.h"
#include "clustering.h"
#include <map>

namespace scene_node::avoidance {

  class ObstacleFieldBuilder {
public:
    ObstacleField buildFromClusters(
      const std::map < std::string, std::vector < ClusterInfo >> & radar_clusters,
      const std::map < std::string, rclcpp::Time > & radar_times,
      const rclcpp::Time & now,
      const ObstacleFieldParams & params) const;

private:
    ObstaclePrimitive toObstacle(
      const ClusterInfo & cluster,
      const std::string & source,
      const rclcpp::Time & stamp,
      const ObstacleFieldParams & params) const;
  };

}  // namespace scene_node::avoidance
