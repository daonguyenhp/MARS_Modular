#pragma once

#include <optional>
#include <vector>

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management {

/** Accumulated undirected graph containing only visibility-supported edges. */
class VisibilityGraph {
 public:
  /** Add a node or reuse the node with the same stable semantic identity. */
  VisibilityNodeId add_or_merge_node(
      const mars::common::Point2D& position,
      VisibilityNodeKind kind,
      std::optional<ObservationId> observation_id = std::nullopt,
      std::optional<OpenPointId> open_point_id = std::nullopt,
      double merge_tolerance = 1.0e-9);

  /** Add an undirected Euclidean-cost edge backed by geometric evidence. */
  bool add_visible_edge(
      VisibilityNodeId first,
      VisibilityNodeId second,
      VisibilityEvidence evidence,
      std::vector<ObservationId> supporting_observations = {});

  /** Read-only lookup operations. */
  const VisibilityNode* find_node(VisibilityNodeId id) const noexcept;
  const VisibilityEdge* find_edge(VisibilityNodeId first,
                                  VisibilityNodeId second) const noexcept;
  /** Find the center node owned by an observation. */
  std::optional<VisibilityNodeId> node_for_observation(
      ObservationId observation_id) const noexcept;
  /** Find the graph node representing an open point. */
  std::optional<VisibilityNodeId> node_for_open_point(
      OpenPointId open_point_id) const noexcept;
  /** Return adjacent node IDs in stable ascending order. */
  std::vector<VisibilityNodeId> neighbors(VisibilityNodeId id) const;

  /** Return all stable graph nodes. */
  const std::vector<VisibilityNode>& nodes() const noexcept;
  /** Return all undirected graph edges. */
  const std::vector<VisibilityEdge>& edges() const noexcept;

 private:
  VisibilityNodeId next_id_{1};
  std::vector<VisibilityNode> nodes_{};
  std::vector<VisibilityEdge> edges_{};
};

}  // namespace mars::graph_bundle_management
