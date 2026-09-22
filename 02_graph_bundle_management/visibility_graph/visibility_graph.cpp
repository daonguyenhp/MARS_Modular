#include "mars/graph_bundle_management/visibility_graph.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "internal/geometry_helpers.hpp"
#include "internal/id_helpers.hpp"

namespace mars::graph_bundle_management {

VisibilityNodeId VisibilityGraph::add_or_merge_node(
    const mars::common::Point2D& position,
    VisibilityNodeKind kind,
    std::optional<ObservationId> observation_id,
    std::optional<OpenPointId> open_point_id,
    double merge_tolerance) {
  internal::require_finite(position, "node position");
  if (!std::isfinite(merge_tolerance) || merge_tolerance < 0.0) {
    throw std::invalid_argument("merge_tolerance must be finite and non-negative");
  }
  if (kind == VisibilityNodeKind::ObservationCenter && !observation_id) {
    throw std::invalid_argument("observation-center nodes require an observation ID");
  }
  if (kind == VisibilityNodeKind::OpenPoint && !open_point_id) {
    throw std::invalid_argument("open-point nodes require an open-point ID");
  }

  for (const auto& node : nodes_) {
    const bool same_stable_identity =
        (observation_id && node.observation_id == observation_id &&
         kind == VisibilityNodeKind::ObservationCenter && node.kind == kind) ||
        (open_point_id && node.open_point_id == open_point_id &&
         kind == VisibilityNodeKind::OpenPoint && node.kind == kind);
    const bool mergeable_anchor =
        kind == VisibilityNodeKind::SharedSightAnchor && node.kind == kind &&
        internal::point_near(node.position, position, merge_tolerance);
    if (same_stable_identity || mergeable_anchor) {
      if (!internal::point_near(node.position, position, merge_tolerance)) {
        throw std::invalid_argument("stable node identity was reused at a new position");
      }
      return node.id;
    }
  }

  nodes_.push_back(
      {next_id_++, position, kind, observation_id, open_point_id});
  return nodes_.back().id;
}

bool VisibilityGraph::add_visible_edge(
    VisibilityNodeId first,
    VisibilityNodeId second,
    VisibilityEvidence evidence,
    std::vector<ObservationId> supporting_observations) {
  // Logic ported from dsfm_polygon_node.py::_graph_insert.
  // Verified against paper Section 4.5; its collision/visibility prerequisite
  // is represented here by mandatory evidence.
  if (first == second) {
    return false;
  }
  const auto* first_node = find_node(first);
  const auto* second_node = find_node(second);
  if (first_node == nullptr || second_node == nullptr) {
    throw std::invalid_argument("edge endpoint does not exist");
  }
  const auto endpoints = internal::canonical_edge(first, second);
  if (find_edge(endpoints.first, endpoints.second) != nullptr) {
    return false;
  }
  std::sort(supporting_observations.begin(), supporting_observations.end());
  supporting_observations.erase(
      std::unique(supporting_observations.begin(), supporting_observations.end()),
      supporting_observations.end());
  edges_.push_back({endpoints.first,
                    endpoints.second,
                    internal::distance(first_node->position, second_node->position),
                    evidence,
                    std::move(supporting_observations)});
  return true;
}

const VisibilityNode* VisibilityGraph::find_node(VisibilityNodeId id) const noexcept {
  for (const auto& node : nodes_) {
    if (node.id == id) {
      return &node;
    }
  }
  return nullptr;
}

const VisibilityEdge* VisibilityGraph::find_edge(VisibilityNodeId first,
                                                 VisibilityNodeId second) const noexcept {
  const auto endpoints = internal::canonical_edge(first, second);
  for (const auto& edge : edges_) {
    if (edge.first == endpoints.first && edge.second == endpoints.second) {
      return &edge;
    }
  }
  return nullptr;
}

std::optional<VisibilityNodeId> VisibilityGraph::node_for_observation(
    ObservationId observation_id) const noexcept {
  for (const auto& node : nodes_) {
    if (node.kind == VisibilityNodeKind::ObservationCenter &&
        node.observation_id == observation_id) {
      return node.id;
    }
  }
  return std::nullopt;
}

std::optional<VisibilityNodeId> VisibilityGraph::node_for_open_point(
    OpenPointId open_point_id) const noexcept {
  for (const auto& node : nodes_) {
    if (node.kind == VisibilityNodeKind::OpenPoint &&
        node.open_point_id == open_point_id) {
      return node.id;
    }
  }
  return std::nullopt;
}

std::vector<VisibilityNodeId> VisibilityGraph::neighbors(VisibilityNodeId id) const {
  if (find_node(id) == nullptr) {
    return {};
  }
  std::vector<VisibilityNodeId> result;
  for (const auto& edge : edges_) {
    if (edge.first == id) {
      result.push_back(edge.second);
    } else if (edge.second == id) {
      result.push_back(edge.first);
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

const std::vector<VisibilityNode>& VisibilityGraph::nodes() const noexcept {
  return nodes_;
}

const std::vector<VisibilityEdge>& VisibilityGraph::edges() const noexcept {
  return edges_;
}

}  // namespace mars::graph_bundle_management
