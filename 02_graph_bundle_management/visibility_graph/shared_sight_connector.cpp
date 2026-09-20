#include "mars/graph_bundle_management/shared_sight_connector.hpp"

#include <cmath>
#include <stdexcept>

#include "internal/geometry_helpers.hpp"

namespace mars::graph_bundle_management {

bool neighbor_sight_supports_point(
    const mars::perception_geometry::NeighborSight& sight,
    const mars::common::Point2D& destination,
    double epsilon) {
  if (!std::isfinite(sight.radius) || sight.radius < 0.0 ||
      !std::isfinite(epsilon) || epsilon < 0.0) {
    throw std::invalid_argument(
        "sight radius and epsilon must be finite and non-negative");
  }
  internal::require_finite(sight.center, "sight center");
  internal::require_finite(destination, "destination");
  if (internal::distance(sight.center, destination) > sight.radius + epsilon) {
    return false;
  }
  const mars::common::Segment2D candidate{sight.center, destination};
  for (const auto& boundary : sight.visible_boundaries) {
    if (!internal::finite(boundary.start) || !internal::finite(boundary.end)) {
      return false;
    }
    if (internal::segments_intersect(candidate, boundary, epsilon) &&
        !internal::point_near(sight.center, boundary.start, epsilon) &&
        !internal::point_near(sight.center, boundary.end, epsilon) &&
        !internal::point_near(destination, boundary.start, epsilon) &&
        !internal::point_near(destination, boundary.end, epsilon)) {
      return false;
    }
  }
  return true;
}

bool neighbor_sights_prove_connection(
    const mars::perception_geometry::NeighborSight& first,
    const mars::perception_geometry::NeighborSight& second,
    double linear_epsilon) {
  if (!std::isfinite(first.radius) || first.radius < 0.0 ||
      !std::isfinite(second.radius) || second.radius < 0.0 ||
      !std::isfinite(linear_epsilon) || linear_epsilon < 0.0) {
    throw std::invalid_argument("sight radii and epsilon must be finite and non-negative");
  }
  internal::require_finite(first.center, "first sight center");
  internal::require_finite(second.center, "second sight center");
  return neighbor_sight_supports_point(first, second.center, linear_epsilon) &&
         neighbor_sight_supports_point(second, first.center, linear_epsilon);
}

}  // namespace mars::graph_bundle_management
