#pragma once

#include "types.hpp"

namespace mars::navigation_decision {

/**
 * MD3.1 Explore.
 *
 * Paper Algorithm 2 / Section 4.3: drive toward the highest-ranked active
 * open point in O_t^g whose connecting path on G_t^e is free. Ranking itself
 * is Module 2. This function only selects among already-ranked, already-routed
 * candidates.
 *
 * Legacy: dsfm_polygon_node.py::_tick_explore (decision order only).
 * Goal-in-sight uses the caller flag (Eq. 3, Section 5: once C_t is within
 * vision range r of g and the segment is free). Arrival uses goal_tolerance
 * so the JetTank path follower and the planner share the same stop band.
 */
ExploreResult explore_step(const Point2D& pose,
                           const Point2D& goal,
                           const std::vector<RankedOpenPoint>& ranked,
                           const std::vector<ReachableRoute>& routes,
                           const NavigationConfig& config,
                           bool goal_visible);

}  // namespace mars::navigation_decision
