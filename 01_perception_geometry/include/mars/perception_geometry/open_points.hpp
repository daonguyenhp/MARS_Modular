#pragma once
#include <mars/perception_geometry/types.hpp>

namespace mars::perception_geometry {
// One radius-r arc midpoint. No ranking, goal, clearance or navigation policy.
// Empty intervals are invalid. Vector API preserves order and original indices.
OpenPoint compute_open_point(const Point2D& center, double radius, const OpenSight& sight,
                             std::optional<std::size_t> sight_index=std::nullopt);
std::vector<OpenPoint> compute_open_points(const Point2D& center, double radius,
                                         const std::vector<OpenSight>& sights);
}  // namespace mars::perception_geometry
