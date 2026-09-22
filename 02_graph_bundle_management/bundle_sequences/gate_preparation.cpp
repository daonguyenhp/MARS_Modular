#include "mars/graph_bundle_management/gate_preparation.hpp"

#include <cmath>
#include <stdexcept>

namespace mars::graph_bundle_management {

GatePreparationResult prepare_gates(const BundleSequence& sequence,
                                    bool gate_contract_enabled,
                                    double linear_epsilon) {
  if (!std::isfinite(linear_epsilon) || linear_epsilon < 0.0) {
    throw std::invalid_argument("linear_epsilon must be finite and non-negative");
  }
  if (!gate_contract_enabled) {
    return {false, GateFailureReason::ContractNotEnabled,
            "The Module 3 portal contract has not been agreed; no gates were manufactured.",
            {}};
  }
  if (sequence.bundles.empty()) {
    return {false, GateFailureReason::EmptySequence,
            "A gate sequence cannot be derived from an empty bundle sequence.", {}};
  }

  // The inspected legacy code contains artificial path-perpendicular and
  // center-link gates, both explicitly excluded by the implementation plan.
  // Until Module 3 defines a supported portal representation, fail visibly.
  return {false, GateFailureReason::UnsupportedGeometry,
          "No agreed visible-boundary-to-portal derivation is available.", {}};
}

}  // namespace mars::graph_bundle_management
