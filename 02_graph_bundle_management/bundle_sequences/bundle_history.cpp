#include "mars/graph_bundle_management/bundle_history.hpp"

#include <stdexcept>
#include <utility>

namespace mars::graph_bundle_management {

void BundleHistory::add(Bundle bundle) {
  // Based on online_bundle_builder.py::BundleHistory with uniqueness enforced
  // for both stable lookup keys, as required by the Module 2 history contract.
  if (by_observation(bundle.observation_id) != nullptr) {
    throw std::invalid_argument("duplicate bundle observation ID");
  }
  if (by_center_node(bundle.center_node_id) != nullptr) {
    throw std::invalid_argument("duplicate bundle center-node ID");
  }
  bundles_.push_back(std::move(bundle));
}

const Bundle* BundleHistory::by_observation(
    ObservationId observation_id) const noexcept {
  for (const auto& bundle : bundles_) {
    if (bundle.observation_id == observation_id) {
      return &bundle;
    }
  }
  return nullptr;
}

const Bundle* BundleHistory::by_center_node(
    VisibilityNodeId center_node_id) const noexcept {
  for (const auto& bundle : bundles_) {
    if (bundle.center_node_id == center_node_id) {
      return &bundle;
    }
  }
  return nullptr;
}

const std::vector<Bundle>& BundleHistory::bundles() const noexcept {
  return bundles_;
}

std::size_t BundleHistory::size() const noexcept {
  return bundles_.size();
}

}  // namespace mars::graph_bundle_management
