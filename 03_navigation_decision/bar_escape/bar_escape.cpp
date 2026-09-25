#include "bar_escape.hpp"

#include <algorithm>
#include <sstream>

#include "internal/geometry.hpp"

namespace mars::navigation_decision {

bool is_blind_alley(ExploreKind kind) noexcept {
  return kind == ExploreKind::BlindAlley;
}

EscapePlan plan_escape(const Point2D& pose,
                       const std::vector<EscapeCandidate>& candidates,
                       const Point2D& entry_pose,
                       const NavigationConfig& config) {
  validate_config(config);
  internal::require_finite(pose, "pose");
  internal::require_finite(entry_pose, "entry_pose");

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
    if (internal::distance(pose, candidate.owner_pose) >
        config.arrival_tolerance) {
      EscapePlan plan;
      plan.kind = EscapeKind::Retreat;
      plan.reason = "DEADEND: return to owner of open point";
      plan.target = candidate.owner_pose;
      plan.retired_id = last_retired;
      std::ostringstream log;
      log << "BAR: return to c[" << candidate.owner_pose.x << ", "
          << candidate.owner_pose.y << "] (owner of open pt ["
          << candidate.point.x << ", " << candidate.point.y << "])";
      plan.log = log.str();
      return plan;
    }
    last_retired = candidate.id;
  }

  if (internal::distance(pose, entry_pose) > config.arrival_tolerance) {
    EscapePlan plan;
    plan.kind = EscapeKind::Retreat;
    plan.reason = "FRONTIER_EXHAUSTED: retreat to entry";
    plan.target = entry_pose;
    plan.retired_id = last_retired;
    std::ostringstream log;
    log << "BAR: retreat to entry [" << entry_pose.x << ", " << entry_pose.y
        << "]";
    plan.log = log.str();
    return plan;
  }

  EscapePlan plan;
  plan.kind = EscapeKind::FrontierExhausted;
  plan.reason = "FRONTIER_EXHAUSTED: already at entry";
  plan.log = "BAR: no unexplored open point left; already at entry";
  return plan;
}

}  // namespace mars::navigation_decision
