#pragma once

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management {

/**
 * Portals are the common edges of a triangulation of C*. C* starts at the
 * bundle Cf of (11) and (12). Each later bundle keeps the segments on the
 * side of angle c_{i-1} c_i c_{i+1} that is at most pi, a segment is added
 * where a bundle is degenerate, and segments of distinct bundles are trimmed
 * until they no longer cross. Center links and path-perpendicular segments
 * are not gates.
 */
GatePreparationResult prepare_gates(const BundleSequence& sequence,
                                    bool gate_contract_enabled,
                                    double linear_epsilon = 1.0e-9);

}  // namespace mars::graph_bundle_management
