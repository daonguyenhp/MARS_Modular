#include <cassert>
#include <cmath>
#include <limits>

#include "laser_geometry.hpp"

int main() {
  mars::online_gazebo::LaserFrame scan;
  scan.angle_min = -0.5;
  scan.angle_increment = 0.1;
  scan.range_min = 0.06;
  scan.range_max = 1.6;
  scan.ranges.assign(11, std::numeric_limits<float>::infinity());
  for (int i = 3; i <= 7; ++i) scan.ranges[static_cast<std::size_t>(i)] = 1.0f;
  const mars::common::Pose2D pose{{3.0, 2.2}, 0.0};
  const auto walls = mars::online_gazebo::scan_to_obstacles(scan, pose, 1.6);
  assert(!walls.empty());
  for (const auto& wall : walls) {
    assert(wall.vertices.size() == 2);
    for (const auto& p : wall.vertices) assert(p.x > 3.0);
  }
  assert(std::abs(mars::online_gazebo::forward_clearance(scan) - 1.0) < 1e-5);
  assert(!mars::online_gazebo::target_visible_in_scan(scan, pose, {4.2, 2.2}, 1.6));
  assert(mars::online_gazebo::target_visible_in_scan(scan, pose, {3.5, 2.2}, 1.6));
  return 0;
}
