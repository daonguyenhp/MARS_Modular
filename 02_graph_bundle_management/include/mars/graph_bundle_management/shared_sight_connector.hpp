#pragma once

#include "mars/perception_geometry/types.hpp"

namespace mars::graph_bundle_management {

/** Prove that one point is in range and unobstructed in a neighbor snapshot. */
bool neighbor_sight_supports_point(
    const mars::perception_geometry::NeighborSight& sight,
    const mars::common::Point2D& point,
    double linear_epsilon = 1.0e-9);

/**
 * Conservatively proves direct center-to-center visibility from two stored
 * neighbor-sight snapshots. No connection is inferred from distance alone.
 */
bool neighbor_sights_prove_connection(
    const mars::perception_geometry::NeighborSight& first,
    const mars::perception_geometry::NeighborSight& second,
    double linear_epsilon = 1.0e-9);

}  // namespace mars::graph_bundle_management
