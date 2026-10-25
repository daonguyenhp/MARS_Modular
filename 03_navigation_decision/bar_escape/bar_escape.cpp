#include "bar_escape.hpp"

#include <algorithm>
#include <sstream>

#include "internal/geometry.hpp"

// Encapsulate the code within a separate space
namespace mars::navigation_decision {

bool is_blind_alley(ExploreKind kind) noexcept {
    return kind == ExploreKind::BlindAlley;
}


/**
    * @brief Plans an escape from a blind alley.
    * 
    * @param pose The current pose of the robot.
    * @param candidates The candidates for escape.
    * @param entry_pose The pose of the entry point.
    * @param config The navigation configuration.
    * @return The escape plan.
*/


EscapePlan plan_escape(const Point2D& pose,
                       const std::vector<EscapeCandidate>& candidates,
                       const Point2D& entry_pose,
                       const NavigationConfig& config) {
  validate_config(config);
  internal::require_finite(pose, "pose");
  internal::require_finite(entry_pose, "entry_pose");

  // Paper Section 1 & Section 4.6 (BAR Problem Definition):
  // (BAR) The robot is in a blind alley region, that is bounded by obstacles and
  // it provides no possible passage for robots except in and out of its path entry,
  // then the robot has to return to some position outside the region (entry_pose)
  // such that the returned path is not longer than the path entry.
  if (internal::distance(pose, entry_pose) > config.arrival_tolerance) {
    EscapePlan plan;
    plan.kind = EscapeKind::Retreat;
    plan.reason = "BAR: return to path entry outside region";
    plan.target = entry_pose;
    std::ostringstream log;
    log << "BAR: return to entry [" << entry_pose.x << ", " << entry_pose.y << "]";
    plan.log = log.str();
    return plan;
  }

  // Already at entry pose: choose best candidate open point outside the alley
  std::vector<EscapeCandidate> remaining = candidates;
  std::sort(remaining.begin(), remaining.end(),
            [](const EscapeCandidate& first, const EscapeCandidate& second) {
              if (first.score != second.score) {
                return first.score > second.score;
              }
              return first.id < second.id;
            });

  std::optional<OpenPointId> last_retired;
  for (const auto& candidate : remaining) {
    internal::require_finite(candidate.point, "escape candidate");
    internal::require_finite(candidate.owner_pose, "owner pose");
    if (internal::distance(pose, candidate.point) > config.arrival_tolerance) {
      EscapePlan plan;
      plan.kind = EscapeKind::Retreat;
      plan.reason = "BAR: explore candidate outside alley";
      plan.target = candidate.point;
      plan.retired_id = last_retired;
      std::ostringstream log;
      log << "BAR: target open point [" << candidate.point.x << ", "
          << candidate.point.y << "] outside alley";
      plan.log = log.str();
      return plan;
    }
    last_retired = candidate.id;
  }

  EscapePlan plan;
  plan.kind = EscapeKind::FrontierExhausted;
  plan.reason = "FRONTIER_EXHAUSTED: already at entry";
  plan.log = "BAR: no unexplored open point left; already at entry";
  return plan;
}

}  // namespace mars::navigation_decision
