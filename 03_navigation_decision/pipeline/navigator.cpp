#include "navigator.hpp"

#include "bar_escape.hpp"
#include "dap_funnel.hpp"
#include "explore.hpp"

namespace mars::navigation_decision {

Navigator::Navigator(NavigationConfig config) : config_(config) {
  validate_config(config_);
}

NavigationDecision Navigator::decide(const NavigationObservation& observation) {
  if (state_ == NavigationState::GoalReached ||
      state_ == NavigationState::Failed) {
    NavigationDecision done;
    done.state = state_;
    done.event = state_ == NavigationState::Failed ? "FAILED" : "GOAL";
    done.log = "already terminal";
    return done;
  }
  if (state_ == NavigationState::Escape) {
    return decide_escape(observation);
  }
  return decide_explore(observation);
}

NavigationDecision Navigator::decide_explore(
    const NavigationObservation& observation) {
  const ExploreResult explored =
      explore_step(observation.pose, observation.goal, observation.ranked,
                   observation.routes, config_, observation.goal_visible);

  if (explored.kind == ExploreKind::GoalReached) {
    state_ = NavigationState::GoalReached;
    NavigationDecision decision;
    decision.state = state_;
    decision.event = "GOAL";
    decision.log = explored.reason;
    return decision;
  }
  if (explored.kind == ExploreKind::GoalVisible) {
    NavigationDecision decision;
    decision.state = NavigationState::Explore;
    decision.event = "GOAL_VISIBLE";
    decision.log = explored.reason;
    decision.selected_target = observation.goal;
    decision.planned_path = explored.path;
    return decision;
  }
  if (explored.kind == ExploreKind::Step) {
    NavigationDecision decision;
    decision.state = NavigationState::Explore;
    decision.event = "EXPLORE";
    decision.log = explored.reason;
    decision.selected_id = explored.target_id;
    decision.selected_target = explored.next_point;
    decision.planned_path = explored.path;
    return decision;
  }

  const EscapePlan plan =
      plan_escape(observation.pose, observation.escape_candidates,
                  observation.entry_pose, config_);
  if (plan.kind == EscapeKind::FrontierExhausted) {
    state_ = NavigationState::Failed;
    NavigationDecision decision;
    decision.state = state_;
    decision.event = "FRONTIER_EXHAUSTED";
    decision.log = plan.log;
    return decision;
  }
  state_ = NavigationState::Escape;
  return_target_ = plan.target;
  NavigationDecision decision;
  decision.state = state_;
  decision.event = "BAR";
  decision.log = plan.log;
  decision.selected_id = plan.retired_id;
  decision.selected_target = plan.target;
  return decision;
}

NavigationDecision Navigator::decide_escape(
    const NavigationObservation& observation) {
  if (!return_target_.has_value() || observation.at_return_target) {
    state_ = NavigationState::Explore;
    return_target_.reset();
    NavigationDecision decision;
    decision.state = state_;
    decision.event = "RETURN_REACHED";
    decision.log = "back to Explore";
    return decision;
  }

  const FunnelResult funnel =
      funnel_path(observation.pose, *return_target_, observation.gates, config_);
  NavigationDecision decision;
  decision.state = NavigationState::Escape;
  decision.selected_target = return_target_;
  if (!funnel.success) {
    decision.event = "FUNNEL_FAIL";
    decision.log = funnel.message;
    return decision;
  }
  decision.event = "FUNNEL";
  decision.planned_path = funnel.path;
  if (!observation.entry_path.empty()) {
    decision.return_leq_entry =
        return_not_longer(observation.entry_path, funnel.path.points,
                          config_.linear_epsilon);
  }
  decision.log = decision.return_leq_entry.has_value()
                     ? (*decision.return_leq_entry
                            ? "DAP/funnel L_return <= L_entry"
                            : "DAP/funnel L_return > L_entry")
                     : "DAP/funnel path";
  return decision;
}

}  // namespace mars::navigation_decision
