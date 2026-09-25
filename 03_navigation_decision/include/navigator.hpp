#pragma once

#include "types.hpp"

namespace mars::navigation_decision {

/**
 * Thin Algorithm 2 state machine. Owns Explore / Escape / Done only.
 * Perception, ranking, visibility graph, and bundle/gate construction stay
 * in Modules 1 and 2. The caller (Module 4) fills NavigationObservation.
 *
 * Legacy call order from dsfm_polygon_node.py, without ROS:
 *   at goal → goal visible → explore step → BAR → funnel → return reached.
 */
class Navigator {
 public:
  explicit Navigator(NavigationConfig config = {});

  NavigationDecision decide(const NavigationObservation& observation);

  NavigationState state() const noexcept { return state_; }
  const std::optional<Point2D>& return_target() const noexcept {
    return return_target_;
  }

 private:
  NavigationDecision decide_explore(const NavigationObservation& observation);
  NavigationDecision decide_escape(const NavigationObservation& observation);

  NavigationConfig config_{};
  NavigationState state_{NavigationState::Explore};
  std::optional<Point2D> return_target_{};
};

}  // namespace mars::navigation_decision
