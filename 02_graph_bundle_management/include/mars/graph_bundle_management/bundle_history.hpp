#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management {

/** Append-only bundle storage indexed by observation and center-node IDs. */
class BundleHistory {
 public:
  /** Add a bundle, rejecting duplicate stable lookup keys. */
  void add(Bundle bundle);
  /** Look up a bundle by observation ID, or return null. */
  const Bundle* by_observation(ObservationId observation_id) const noexcept;
  /** Look up a bundle by visibility center-node ID, or return null. */
  const Bundle* by_center_node(VisibilityNodeId center_node_id) const noexcept;
  /** Return all bundles in insertion order. */
  const std::vector<Bundle>& bundles() const noexcept;
  /** Return the number of stored observations. */
  std::size_t size() const noexcept;

 private:
  std::vector<Bundle> bundles_{};
};

}  // namespace mars::graph_bundle_management
