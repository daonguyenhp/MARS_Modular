#pragma once

#include <algorithm>
#include <utility>

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management::internal {

inline std::pair<VisibilityNodeId, VisibilityNodeId> canonical_edge(
    VisibilityNodeId first, VisibilityNodeId second) noexcept {
  return std::minmax(first, second);
}

}  // namespace mars::graph_bundle_management::internal
