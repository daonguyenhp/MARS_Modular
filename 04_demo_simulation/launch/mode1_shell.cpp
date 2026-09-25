#include "mode1_shell.hpp"

#include <cmath>
#include <utility>

#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/perception_geometry/perception_pipeline.hpp"

namespace mars::demo_simulation {
namespace {

double distance(mars::common::Point2D a, mars::common::Point2D b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

Mode1Tick run_mode1_tick(const Mode1Scenario& scenario, const LoadedMap& map) {
  mars::navigation_decision::NavigationConfig config;
  const auto perception = mars::perception_geometry::perceive(
      scenario.start, config.vision_radius, map.obstacles);

  mars::graph_bundle_management::GraphBundleUpdate update;
  update.observation_id = 1;
  update.current_pose = {scenario.start, 0.0};
  update.goal = scenario.goal;
  update.perception = perception;

  mars::graph_bundle_management::GraphBundleManager manager;
  const auto summary = manager.update(update);

  mars::navigation_decision::NavigationObservation observation;
  observation.pose = scenario.start;
  observation.goal = scenario.goal;
  observation.goal_visible =
      distance(scenario.start, scenario.goal) <= config.vision_radius;
  observation.ranked = manager.ranked_open_points();
  observation.entry_pose = scenario.start;

  for (const auto& ranked : observation.ranked) {
    mars::navigation_decision::EscapeCandidate candidate;
    candidate.id = ranked.id;
    candidate.point = ranked.point;
    candidate.score = ranked.score;
    candidate.owner_pose = scenario.start;
    if (const auto* record = manager.open_point_memory().find(ranked.id)) {
      candidate.owner_pose = record->source_center;
    }
    observation.escape_candidates.push_back(candidate);

    const auto prepared = manager.prepare_route(ranked.id);
    if (!prepared.success) {
      continue;
    }
    mars::navigation_decision::ReachableRoute route;
    route.id = ranked.id;
    for (const auto node_id : prepared.skeleton_path.node_ids) {
      if (const auto* node = manager.visibility_graph().find_node(node_id)) {
        route.skeleton.push_back(node->position);
      }
    }
    if (route.skeleton.size() >= 2) {
      observation.routes.push_back(std::move(route));
    }
  }

  mars::navigation_decision::Navigator navigator(config);
  const auto decision = navigator.decide(observation);

  Mode1Tick tick;
  tick.wall_count = map.obstacles.size();
  tick.closed_sights = perception.closed_sights.size();
  tick.open_sights = perception.open_sights.size();
  tick.open_points = perception.open_points.size();
  tick.active_open_points = summary.active_open_point_count;
  tick.state = decision.state;
  tick.event = decision.event;
  tick.log = decision.log;
  return tick;
}

}  // namespace mars::demo_simulation
