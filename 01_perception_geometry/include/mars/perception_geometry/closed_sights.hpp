#pragma once
#include <mars/perception_geometry/types.hpp>

namespace mars::perception_geometry {
// Input must contain occlusion-resolved fragments (normally computed by
// compute_neighbor_sight). Union of their minor angular projections, sorted
// by start; merged sights retain their supporting visible fragments.
std::vector<ClosedSight> compute_closed_sights(const NeighborSight& neighbor);
}  // namespace mars::perception_geometry
