#pragma once

#include "types.hpp"

namespace mars::navigation_decision {

/**
 * MD3.3 DAP / funnel.
 *
 * Paper: the escape path is a δ-approximate shortest path through the
 * sequence of bundles of line segments. Corners are cut only at bundle
 * vertices whose interior angle is < π, which is the taut-string / funnel
 * property of Lee and Preparata. The path stays inside the explored free
 * space S_t^e, and its length must satisfy L_return ≤ L_entry.
 *
 * Gates are the Module 2 portal contract (left/right along travel). This
 * module does not build bundles or invent gates. Empty gates mean the
 * corridor is unconstrained, so the taut path is the straight segment
 * (robot DIRECT case: target already in line of sight).
 *
 * JetTank consumes the polyline in the map frame; (v, w) tracking belongs
 * to Module 4 / the path follower, not here.
 */
FunnelResult funnel_path(const Point2D& start,
                         const Point2D& goal,
                         const std::vector<Gate>& gates,
                         const NavigationConfig& config = {});

bool return_not_longer(const std::vector<Point2D>& entry,
                       const std::vector<Point2D>& ret,
                       double epsilon = 1.0e-9);

}  // namespace mars::navigation_decision
