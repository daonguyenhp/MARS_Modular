#pragma once

#include <vector>

#include "mars/common/geometry_types.hpp"

namespace mars::online_gazebo {

struct LaserFrame {
  std::vector<float> ranges{};
  double angle_min{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

// Only currently measured, finite returns. No world-map oracle is consulted.
std::vector<mars::common::Polygon2D> scan_to_obstacles(
    const LaserFrame& scan, mars::common::Pose2D pose,
    double vision_radius);

bool target_visible_in_scan(const LaserFrame& scan,
                            mars::common::Pose2D pose,
                            mars::common::Point2D target,
                            double vision_radius);

double forward_clearance(const LaserFrame& scan);

}  // namespace mars::online_gazebo
