#pragma once

#include <map>
#include <optional>
#include <vector>

#include "mars/graph_bundle_management/bundle_history.hpp"
#include "mars/graph_bundle_management/open_point_memory.hpp"
#include "mars/graph_bundle_management/ranking.hpp"
#include "mars/graph_bundle_management/visibility_graph.hpp"

namespace mars::graph_bundle_management {

/** Convenience composition of all ROS-independent Module 2 components. */
class GraphBundleManager {
 public:
  /** Construct and validate a manager configuration. */
  explicit GraphBundleManager(GraphBundleConfig config = {});

  /** Accept one new observation and update all accumulated state. */
  UpdateSummary update(const GraphBundleUpdate& input);

  /** Return the ranking computed during the latest update. */
  std::vector<RankedOpenPoint> ranked_open_points() const;
  /** Return the accumulated read-only visibility graph. */
  const VisibilityGraph& visibility_graph() const noexcept;
  /** Return the accumulated read-only open-point memory. */
  const OpenPointMemory& open_point_memory() const noexcept;
  /** Return the append-only bundle history. */
  const BundleHistory& bundle_history() const noexcept;

  /** Prepare a route using the search policy configured at construction. */
  RoutePreparationResult prepare_route(OpenPointId target) const;
  /** Prepare a route using an explicit search policy. */
  RoutePreparationResult prepare_route(
      OpenPointId target,
      GraphSearchPolicy policy) const;

 private:
  GraphBundleConfig config_;
  OpenPointMemory open_point_memory_;
  VisibilityGraph visibility_graph_{};
  BundleHistory bundle_history_{};
  std::vector<RankedOpenPoint> ranked_open_points_{};
  std::map<ObservationId, mars::perception_geometry::NeighborSight> sightings_{};
  std::optional<VisibilityNodeId> current_center_node_id_{};
};

}  // namespace mars::graph_bundle_management
