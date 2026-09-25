#pragma once

#include "types.hpp"

namespace mars::navigation_decision {

/**
 * MD3.2 Blind Alley Region (BAR).
 *
 * Paper: a BAR is an explored neighborhood that offers no remaining open
 * direction toward the goal. Algorithm 2 then leaves the region along the
 * stored sequences of bundles rather than inventing a new frontier.
 *
 * Legacy: dsfm_polygon_node.py::_set_deadend_return. Retreat to the owner
 * concurrent point of the best remaining open point. If the robot already
 * stands on that owner, retire the point (it is unreachable from its own
 * owner) and try the next. If none remain, retreat to the entry pose C_0.
 */
bool is_blind_alley(ExploreKind kind) noexcept;

EscapePlan plan_escape(const Point2D& pose,
                       const std::vector<EscapeCandidate>& candidates,
                       const Point2D& entry_pose,
                       const NavigationConfig& config);

}  // namespace mars::navigation_decision
