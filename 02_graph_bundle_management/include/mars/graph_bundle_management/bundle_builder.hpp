#pragma once

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management {

/** Build a deterministic paper bundle from Module 1 visible boundaries. */
Bundle build_bundle(
    const mars::perception_geometry::PerceptionResult& perception,
    ObservationId observation_id,
    VisibilityNodeId center_node_id,
    double linear_tolerance = 1.0e-9);

}  // namespace mars::graph_bundle_management
