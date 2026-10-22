#pragma once
#include <mars/perception_geometry/types.hpp>

namespace mars::perception_geometry {
// One open point per free arc: its midpoint. That is the way through the
// opening. Points toward the arc edges lie against the wall and are omitted.
// Empty intervals are invalid. Vector API keeps sight order.
OpenPoint compute_open_point(const Point2D& center, double radius, const OpenSight& sight,
                             std::optional<std::size_t> sight_index=std::nullopt);
std::vector<OpenPoint> compute_open_points(const Point2D& center, double radius,
                                         const std::vector<OpenSight>& sights);
}  // namespace mars::perception_geometry
