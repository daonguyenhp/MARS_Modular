#pragma once
#include <mars/perception_geometry/types.hpp>

namespace mars::perception_geometry {
// Simple filled polygons or two-vertex thin walls. Center must be outside all
// filled polygons and more than linear_epsilon from every nondegenerate edge.
// Invalid topology/observer placement throws std::invalid_argument.
NeighborSight compute_neighbor_sight(const Point2D& center, double radius,
                                     const std::vector<Polygon2D>& obstacles);
}  // namespace mars::perception_geometry
