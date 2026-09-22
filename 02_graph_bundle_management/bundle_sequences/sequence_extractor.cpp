#include "mars/graph_bundle_management/sequence_extractor.hpp"

#include <algorithm>

namespace mars::graph_bundle_management {

BundleSequenceResult extract_bundle_sequence(
    const GraphPath& graph_path,
    const VisibilityGraph& graph,
    const BundleHistory& bundle_history,
    BundleSequenceDirection direction) {
  if (graph_path.node_ids.empty()) {
    return {false, SequenceFailureReason::EmptyGraphPath,
            "The graph path is empty.", {}};
  }

  std::vector<VisibilityNodeId> nodes = graph_path.node_ids;
  if (direction == BundleSequenceDirection::TargetToCurrent) {
    std::reverse(nodes.begin(), nodes.end());
  }
  BundleSequence sequence;
  sequence.skeleton_node_ids = nodes;
  sequence.direction = direction;

  // This intentionally fixes BundleHistory::extract_raw_sequence in the legacy
  // implementation, which sorted ascending even for reverse traversal.
  for (const auto node_id : nodes) {
    const auto* node = graph.find_node(node_id);
    if (node == nullptr || node->kind != VisibilityNodeKind::ObservationCenter ||
        !node->observation_id) {
      continue;
    }
    const ObservationId observation_id = *node->observation_id;
    if (!sequence.observation_ids.empty() &&
        sequence.observation_ids.back() == observation_id) {
      continue;
    }
    const auto* bundle = bundle_history.by_observation(observation_id);
    if (bundle == nullptr) {
      return {false, SequenceFailureReason::MissingBundle,
              "A route observation has no stored bundle.", {}};
    }
    if (bundle->degenerate) {
      return {false, SequenceFailureReason::DegenerateBundle,
              "A route observation has a degenerate bundle.", {}};
    }
    sequence.observation_ids.push_back(observation_id);
    sequence.bundles.push_back(*bundle);
  }
  return {true, SequenceFailureReason::None, {}, std::move(sequence)};
}

}  // namespace mars::graph_bundle_management
