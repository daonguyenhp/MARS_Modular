#pragma once

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management {

/**
 * Prepare portals only when the Module 3 gate contract is enabled. The current
 * repository has no agreed portal representation, so enabled calls return an
 * explicit unsupported-geometry failure instead of inventing gates.
 */
GatePreparationResult prepare_gates(const BundleSequence& sequence,
                                    bool gate_contract_enabled,
                                    double linear_epsilon = 1.0e-9);

}  // namespace mars::graph_bundle_management
