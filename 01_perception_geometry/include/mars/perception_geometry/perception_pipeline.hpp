#pragma once
#include <mars/perception_geometry/types.hpp>

namespace mars::perception_geometry {
// Stateless, ROS-independent perception of a complete synthetic obstacle map.
// No goal, history, ranking, robot footprint, or motion control is involved.
PerceptionResult perceive(const Point2D& center, double radius,
                          const std::vector<Polygon2D>& obstacles);
}  // namespace mars::perception_geometry
