#include "explore.hpp"

#include <algorithm>

#include "internal/geometry.hpp"

namespace mars::navigation_decision {

ExploreResult explore_step(const Point2D& pose,
                           const Point2D& goal,
                           const std::vector<RankedOpenPoint>& ranked,
                           const std::vector<ReachableRoute>& routes,
                           const NavigationConfig& config,
                           bool goal_visible) {
  validate_config(config);
  internal::require_finite(pose, "pose");
  internal::require_finite(goal, "goal");

  if (internal::distance(pose, goal) <= config.goal_tolerance) {
    return {ExploreKind::GoalReached, "at goal", std::nullopt, std::nullopt, {}};
  }

  // Paper Section 5 Eq. 3: if g lies in the current vision disk and the
  // caller has confirmed a free segment, take it before frontier bookkeeping.
  if (goal_visible) {
    PlannedPath path;
    path.points = {pose, goal};
    path.length = internal::distance(pose, goal);
    return {ExploreKind::GoalVisible, "goal visible, go direct", std::nullopt,
            goal, path};
  }

  if (ranked.empty()) {
    return {ExploreKind::BlindAlley, "global open set empty", std::nullopt,
            std::nullopt, {}};
  }

  std::vector<RankedOpenPoint> ordered = ranked;
  std::sort(ordered.begin(), ordered.end(),
            [](const RankedOpenPoint& first, const RankedOpenPoint& second) {
              if (first.score != second.score) {
                return first.score > second.score;
              }
              return first.id < second.id;
            });

  for (const auto& candidate : ordered) {
    internal::require_finite(candidate.point, "ranked open point");
    const ReachableRoute* route = nullptr;
    for (const auto& item : routes) {
      if (item.id == candidate.id) {
        route = &item;
        break;
      }
    }
    if (route == nullptr || route->skeleton.size() < 2) {
      continue;
    }
    const auto points = cap_last_hop(route->skeleton, config.max_step);
    // A multi-hop cap is measured from an older vertex. When that point
    // falls back inside goal_tolerance of the robot, the follower retraces
    // the detour and stops where it already is. Skip it and try the next
    // frontier that still leaves a real step.
    if (route->skeleton.size() >= 3 &&
        internal::distance(pose, points.back()) <= config.goal_tolerance) {
      continue;
    }
    ExploreResult result;
    result.kind = ExploreKind::Step;
    result.reason = "explore toward highest-rank reachable open point";
    result.target_id = candidate.id;
    result.next_point = candidate.point;
    result.path.points = points;
    result.path.length = path_length(result.path.points);
    return result;
  }

  return {ExploreKind::BlindAlley, "no wall-free path from any open point",
          std::nullopt, std::nullopt, {}};
}

}  // namespace mars::navigation_decision
