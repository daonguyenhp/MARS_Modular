#pragma once

#include "mars/graph_bundle_management/bundle_history.hpp"
#include "mars/graph_bundle_management/visibility_graph.hpp"

namespace mars::graph_bundle_management {

/** Resolve graph center nodes to nondegenerate bundles in requested travel order. */
BundleSequenceResult extract_bundle_sequence(
    const GraphPath& graph_path,
    const VisibilityGraph& graph,
    const BundleHistory& bundle_history,
    BundleSequenceDirection direction = BundleSequenceDirection::CurrentToTarget);

}  // namespace mars::graph_bundle_management
