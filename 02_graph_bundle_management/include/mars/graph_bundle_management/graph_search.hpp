#pragma once

#include <optional>

#include "mars/graph_bundle_management/visibility_graph.hpp"

namespace mars::graph_bundle_management {

/** Find a deterministic path minimizing edge count. */
std::optional<GraphPath> breadth_first_path(const VisibilityGraph& graph,
                                            VisibilityNodeId start,
                                            VisibilityNodeId goal);

/** Find a deterministic Dijkstra path minimizing stored geometric edge cost. */
std::optional<GraphPath> shortest_cost_path(const VisibilityGraph& graph,
                                            VisibilityNodeId start,
                                            VisibilityNodeId goal);

}  // namespace mars::graph_bundle_management
