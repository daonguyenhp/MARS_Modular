#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "bar_escape.hpp"
#include "dap_funnel.hpp"
#include "explore.hpp"
#include "navigator.hpp"

namespace mnd = mars::navigation_decision;
namespace mgbm = mars::graph_bundle_management;
namespace mc = mars::common;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

mgbm::RankedOpenPoint ranked(mgbm::OpenPointId id, mc::Point2D point,
                             double score) {
  mgbm::RankedOpenPoint item;
  item.id = id;
  item.point = point;
  item.score = score;
  return item;
}

mnd::ReachableRoute route(mgbm::OpenPointId id, mc::Point2D from,
                          mc::Point2D to) {
  return {id, {from, to}};
}

mgbm::Gate gate(double lx, double ly, double rx, double ry) {
  mgbm::Gate item;
  item.left = {lx, ly};
  item.right = {rx, ry};
  item.valid = true;
  return item;
}

void test_explore() {
  mnd::NavigationConfig config;
  const auto step = mnd::explore_step(
      {0.0, 0.0}, {10.0, 0.0},
      {ranked(2, {3.0, 0.0}, 0.9), ranked(1, {1.0, 0.0}, 0.2)},
      {route(2, {0.0, 0.0}, {3.0, 0.0})}, config, false);
  require(step.kind == mnd::ExploreKind::Step, "did not pick a reachable point");
  require(step.target_id && *step.target_id == 2, "did not pick highest rank");
  require(std::abs(step.path.points.back().x - 0.70) < 1.0e-9,
          "JetTank max_step 0.70 m was not applied");

  const auto blocked = mnd::explore_step(
      {0.0, 0.0}, {10.0, 0.0}, {ranked(2, {3.0, 0.0}, 0.9)}, {}, config, false);
  require(blocked.kind == mnd::ExploreKind::BlindAlley,
          "unreachable open set was not a BAR");

  const auto visible = mnd::explore_step(
      {0.0, 0.0}, {0.4, 0.0}, {}, {}, config, true);
  require(visible.kind == mnd::ExploreKind::GoalVisible,
          "goal in sight must beat an empty frontier");

  // hard_alley frame 2: the best open point routes back through the previous
  // pose, and the capped end sits inside the follower stop band.
  mnd::ReachableRoute detour;
  detour.id = 1;
  detour.skeleton = {
      {0.946868, 0.78156}, {0.943061, 0.158472}, {0.943908, 1.00847}};
  mnd::ReachableRoute forward;
  forward.id = 2;
  forward.skeleton = {{0.946868, 0.78156}, {0.356377, 1.39297}};
  const mc::Point2D stuck{0.946868, 0.78156};
  const auto recovered = mnd::explore_step(
      stuck, {0.35, 3.40},
      {ranked(1, {0.943908, 1.00847}, 5.14),
       ranked(2, {0.356377, 1.39297}, 2.34)},
      {detour, forward}, config, false);
  require(recovered.kind == mnd::ExploreKind::Step,
          "a forward frontier was not used after the collapsed hop");
  require(recovered.target_id && *recovered.target_id == 2,
          "collapsed backward hop was still selected");
  require(std::hypot(recovered.path.points.back().x - stuck.x,
                      recovered.path.points.back().y - stuck.y) >
              config.goal_tolerance,
          "recovered step does not leave the stop band");

  const auto only_detour = mnd::explore_step(
      stuck, {0.35, 3.40}, {ranked(1, {0.943908, 1.00847}, 5.14)}, {detour},
      config, false);
  require(only_detour.kind == mnd::ExploreKind::BlindAlley,
          "a zero-progress detour was issued to the follower");
}

void test_bar() {
  mnd::NavigationConfig config;
  mnd::EscapeCandidate owned{1, {1.5, 0.0}, {1.0, 0.0}, 0.8};
  const auto retreat =
      mnd::plan_escape({3.0, 0.0}, {owned}, {0.0, 0.0}, config);
  require(retreat.kind == mnd::EscapeKind::Retreat, "BAR did not retreat");
  require(retreat.target && retreat.target->x == 1.0, "did not use owner pose");

  const auto skip_owner =
      mnd::plan_escape({1.0, 0.0}, {owned}, {0.0, 0.0}, config);
  require(skip_owner.target && skip_owner.target->x == 0.0,
          "standing on the owner must fall through to the entry");

  const auto exhausted =
      mnd::plan_escape({0.0, 0.0}, {}, {0.0, 0.0}, config);
  require(exhausted.kind == mnd::EscapeKind::FrontierExhausted,
          "entry with no candidates must be exhausted");
}

void test_funnel() {
  mnd::NavigationConfig config;
  const auto straight = mnd::funnel_path(
      {0.0, 0.0}, {4.0, 0.0},
      {gate(1.0, 1.0, 1.0, -1.0), gate(2.0, 1.0, 2.0, -1.0),
       gate(3.0, 1.0, 3.0, -1.0)},
      config);
  require(straight.success, "straight corridor failed");
  require(std::abs(straight.path.length - 4.0) < 1.0e-6, "straight length");
  require(straight.path.points.size() == 2, "straight path invented a bend");

  const std::vector<mgbm::Gate> corner = {
      gate(1.0, 1.0, 1.0, -1.0), gate(2.0, 1.0, 2.0, -1.0),
      gate(2.0, 2.0, 3.0, 2.0), gate(2.0, 3.0, 3.0, 3.0)};
  const auto taut =
      mnd::funnel_path({0.0, 0.0}, {2.5, 4.0}, corner, config);
  require(taut.success, "corner corridor failed");
  const double expected = std::hypot(2.0, 1.0) + std::hypot(0.5, 3.0);
  require(std::abs(taut.path.length - expected) < 1.0e-4,
          "corner was not taut");
  require(taut.path.points.size() == 3, "corner must bend once");
  require(std::abs(taut.path.points[1].x - 2.0) < 1.0e-4 &&
              std::abs(taut.path.points[1].y - 1.0) < 1.0e-4,
          "bend is not the inside corner");

  const std::vector<mc::Point2D> zigzag = {
      {0.0, 0.0}, {1.0, 0.8}, {2.0, -0.8}, {3.0, 0.8}, {4.0, 0.0}};
  require(mnd::return_not_longer(zigzag, straight.path.points),
          "paper L_return <= L_entry failed on a zigzag entry");
}

void test_navigator() {
  mnd::Navigator navigator;
  mnd::NavigationObservation observation;
  observation.pose = {0.0, 0.0};
  observation.goal = {5.0, 0.0};
  observation.ranked = {ranked(1, {2.0, 0.0}, 1.0)};
  observation.routes = {route(1, {0.0, 0.0}, {2.0, 0.0})};
  observation.entry_pose = {0.0, 0.0};
  observation.entry_path = {
      {0.0, 0.0}, {0.5, 0.4}, {1.0, -0.4}, {2.0, 0.0}};

  auto decision = navigator.decide(observation);
  require(decision.event == "EXPLORE", "first tick was not Explore");

  observation.pose = {2.0, 0.0};
  observation.ranked.clear();
  observation.routes.clear();
  observation.escape_candidates = {{1, {2.0, 0.0}, {0.0, 0.0}, 1.0}};
  decision = navigator.decide(observation);
  require(decision.event == "BAR", "dead-end was not BAR");
  require(navigator.state() == mnd::NavigationState::Escape, "state not Escape");

  observation.gates = {gate(1.5, 0.4, 1.5, -0.4), gate(0.8, 0.4, 0.8, -0.4)};
  decision = navigator.decide(observation);
  require(decision.event == "FUNNEL", "escape tick was not funnel");
  require(decision.return_leq_entry && *decision.return_leq_entry,
          "funnel return longer than entry");

  observation.at_return_target = true;
  decision = navigator.decide(observation);
  require(decision.event == "RETURN_REACHED", "did not resume Explore");
}

void test_config() {
  mnd::NavigationConfig bad;
  bad.max_step = 2.0;
  bad.vision_radius = 0.85;
  bool rejected = false;
  try {
    mnd::validate_config(bad);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "max_step >= 2 r must be rejected");
}

}  // namespace

int main() {
  try {
    test_explore();
    test_bar();
    test_funnel();
    test_navigator();
    test_config();
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "mars_navigation_decision tests passed\n";
  return EXIT_SUCCESS;
}
