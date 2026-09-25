#include "mars/graph_bundle_management/bundle_builder.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "internal/geometry_helpers.hpp"

namespace mars::graph_bundle_management {

Bundle build_bundle(
    const mars::perception_geometry::PerceptionResult& perception,
    ObservationId observation_id,
    VisibilityNodeId center_node_id,
    double linear_tolerance) {
  // Logic ported from paper_bundle_tools.py::extract_paper_bundle and
  // online_bundle_builder.py::build_bundle. Verified against paper Sections
  // 3.1-3.2. Module 1 already supplies the occlusion-resolved boundaries, so
  // ray casting and map ownership are omitted.
  if (!std::isfinite(linear_tolerance) || linear_tolerance < 0.0) {
    throw std::invalid_argument("linear_tolerance must be finite and non-negative");
  }
  const auto center = perception.neighbor_sight.center;
  internal::require_finite(center, "neighbor_sight.center");

  std::vector<mars::common::Point2D> vertices;
  std::vector<mars::common::Segment2D> obstacle_edges;
  for (const auto& boundary : perception.neighbor_sight.visible_boundaries) {
    internal::require_finite(boundary.start, "visible boundary start");
    internal::require_finite(boundary.end, "visible boundary end");
    if (internal::point_near(boundary.start, boundary.end, linear_tolerance)) {
      continue;
    }
    obstacle_edges.push_back(boundary);
    for (const auto endpoint : {boundary.start, boundary.end}) {
      if (internal::point_near(center, endpoint, linear_tolerance)) {
        continue;
      }
      const bool duplicate = std::any_of(
          vertices.begin(), vertices.end(), [&](const auto& existing) {
            return internal::point_near(existing, endpoint, linear_tolerance);
          });
      if (!duplicate) {
        vertices.push_back(endpoint);
      }
    }
  }

  const auto normalized_angle = [&](const mars::common::Point2D& point) {
    double angle = std::atan2(point.y - center.y, point.x - center.x);
    if (angle < 0.0) {
      angle += 2.0 * std::acos(-1.0);
    }
    return angle;
  };
  std::sort(vertices.begin(), vertices.end(), [&](const auto& first,
                                                   const auto& second) {
    const double first_angle = normalized_angle(first);
    const double second_angle = normalized_angle(second);
    if (first_angle != second_angle) {
      return first_angle < second_angle;
    }
    const double first_distance = internal::squared_distance(first, center);
    const double second_distance = internal::squared_distance(second, center);
    if (first_distance != second_distance) {
      return first_distance < second_distance;
    }
    if (first.x != second.x) {
      return first.x < second.x;
    }
    return first.y < second.y;
  });

  std::vector<mars::common::Segment2D> segments;
  segments.reserve(vertices.size());
  for (const auto& vertex : vertices) {
    segments.push_back({center, vertex});
  }
  // Paper Section 3.1 treats a point (zero segments) as the degenerate case;
  // one or more concurrent line segments form a nondegenerate bundle.
  const bool degenerate = segments.empty();
  return {observation_id, center_node_id, center, std::move(vertices),
          std::move(segments), degenerate, std::move(obstacle_edges)};
}

}  // namespace mars::graph_bundle_management
