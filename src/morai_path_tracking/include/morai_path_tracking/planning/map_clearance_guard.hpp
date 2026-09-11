#pragma once
#include <cmath>
#include <geometry_msgs/PointStamped.h>

namespace morai_path_tracking {
// Invalid/unknown data never relaxes the original guard. Freshness is checked
// by the controller on the synchronized path/odometry generation as a whole.
inline double selectMapClearance(double original,
    const geometry_msgs::PointStamped& map, const ros::Time& generation,
    const std::string& frame) {
  return !generation.isZero() && map.header.stamp == generation &&
      map.header.frame_id == frame && map.point.y == 1.0 &&
      std::isfinite(map.point.x) && std::abs(map.point.x) < 10.0 &&
      std::isfinite(map.point.z) && map.point.z >= 0.0 && map.point.z <= 50.0
      ? map.point.x : original;
}
}  // namespace morai_path_tracking
