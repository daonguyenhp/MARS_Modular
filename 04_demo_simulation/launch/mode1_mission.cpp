#include "mode1_mission.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/perception_geometry/perception_pipeline.hpp"
#include "navigator.hpp"
#include "path_follower.hpp"

namespace mars::demo_simulation {
namespace {

double distance(mars::common::Point2D from, mars::common::Point2D to) {
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  return std::sqrt(dx * dx + dy * dy);
}

double cross(mars::common::Point2D a, mars::common::Point2D b,
             mars::common::Point2D c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool segments_cross(mars::common::Point2D a, mars::common::Point2D b,
                    mars::common::Point2D c, mars::common::Point2D d) {
  const double ab_c = cross(a, b, c);
  const double ab_d = cross(a, b, d);
  const double cd_a = cross(c, d, a);
  const double cd_b = cross(c, d, b);
  const bool splits_ab = (ab_c > 0.0 && ab_d < 0.0) || (ab_c < 0.0 && ab_d > 0.0);
  const bool splits_cd = (cd_a > 0.0 && cd_b < 0.0) || (cd_a < 0.0 && cd_b > 0.0);
  if (!splits_ab || !splits_cd) {
    return false;
  }
  constexpr double kTouch = 1.0e-6;
  const double along_ab = cd_a / (cd_a - cd_b);
  const double along_cd = ab_c / (ab_c - ab_d);
  return along_ab > kTouch && along_ab < 1.0 - kTouch && along_cd > kTouch &&
         along_cd < 1.0 - kTouch;
}

bool path_crosses_wall(const std::vector<mars::common::Point2D>& points,
                       const LoadedMap& map) {
  for (std::size_t index = 1; index < points.size(); ++index) {
    for (const auto& obstacle : map.obstacles) {
      const auto& vertices = obstacle.vertices;
      if (vertices.size() < 2) {
        continue;
      }
      for (std::size_t vertex = 0; vertex < vertices.size(); ++vertex) {
        const auto& end = vertices[(vertex + 1) % vertices.size()];
        if (segments_cross(points[index - 1], points[index], vertices[vertex],
                           end)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool goal_in_sight(mars::common::Point2D pose, mars::common::Point2D goal,
                   double radius, const LoadedMap& map) {
  if (distance(pose, goal) > radius) {
    return false;
  }
  for (const auto& obstacle : map.obstacles) {
    const auto& vertices = obstacle.vertices;
    if (vertices.size() < 2) {
      continue;
    }
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      const auto& end = vertices[(i + 1) % vertices.size()];
      if (segments_cross(pose, goal, vertices[i], end)) {
        return false;
      }
    }
  }
  return true;
}

MarkerFrame make_frame(
    std::size_t index, const mars::common::Pose2D& pose, double radius,
    const mars::perception_geometry::PerceptionResult& perception,
    const mars::graph_bundle_management::GraphBundleManager& manager,
    const mars::navigation_decision::NavigationDecision& decision) {
  MarkerFrame frame;
  frame.index = index;
  frame.pose = pose.position;
  frame.yaw = pose.yaw;
  frame.vision_radius = radius;
  frame.event = decision.event;
  for (const auto& sight : perception.closed_sights) {
    frame.closed.push_back({sight.interval.start, sight.interval.sweep});
  }
  for (const auto& sight : perception.open_sights) {
    frame.open.push_back({sight.interval.start, sight.interval.sweep});
  }
  for (const auto& point : perception.open_points) {
    frame.open_points.push_back(point.point);
  }
  frame.graph_nodes = manager.visibility_graph().nodes().size();
  frame.graph_edges = manager.visibility_graph().edges().size();
  for (const auto& edge : manager.visibility_graph().edges()) {
    const auto* first = manager.visibility_graph().find_node(edge.first);
    const auto* second = manager.visibility_graph().find_node(edge.second);
    if (first == nullptr || second == nullptr) {
      continue;
    }
    frame.graph.push_back({first->position, second->position});
  }
  frame.planned_path = decision.planned_path.points;
  frame.bundles = manager.bundle_history().size();
  for (const auto& bundle : manager.bundle_history().bundles()) {
    frame.bundle_centers.push_back(bundle.concurrent_point);
  }
  if (decision.event == "FUNNEL") {
    frame.funnel = decision.planned_path.points;
  }
  return frame;
}

}  // namespace

MissionResult run_mode1_mission(const Mode1Scenario& scenario,
                                const LoadedMap& map, MissionConfig limits) {
  mars::navigation_decision::NavigationConfig config;
  mars::graph_bundle_management::GraphBundleConfig bundle_config;
  bundle_config.gate_contract_enabled = true;
  mars::graph_bundle_management::GraphBundleManager manager(bundle_config);
  mars::navigation_decision::Navigator navigator(config);
  FakeOdom odom{{scenario.start, 0.0}};
  PathFollower follower;

  MissionResult mission;
  mission.trail.push_back(scenario.start);
  std::vector<mars::common::Point2D> entry_path{scenario.start};
  int stalls = 0;
  mars::graph_bundle_management::ObservationId observation_id = 0;

  for (int step = 0; step < limits.max_observations; ++step) {
    ++observation_id;
    const mars::common::Pose2D pose = odom.pose();
    const auto perception = mars::perception_geometry::perceive(
        pose.position, config.vision_radius, map.obstacles);

    mars::graph_bundle_management::GraphBundleUpdate update;
    update.observation_id = observation_id;
    update.current_pose = pose;
    update.goal = scenario.goal;
    update.perception = perception;
    manager.update(update);

    mars::navigation_decision::NavigationObservation observation;
    observation.pose = pose.position;
    observation.goal = scenario.goal;
    observation.goal_visible =
        goal_in_sight(pose.position, scenario.goal, config.vision_radius, map);
    observation.entry_pose = scenario.start;
    observation.entry_path = entry_path;
    observation.ranked = manager.ranked_open_points();
    if (navigator.return_target().has_value()) {
      observation.at_return_target =
          distance(pose.position, *navigator.return_target()) <=
          config.arrival_tolerance;
    }
    for (const auto& ranked : observation.ranked) {
      mars::navigation_decision::EscapeCandidate candidate;
      candidate.id = ranked.id;
      candidate.point = ranked.point;
      candidate.score = ranked.score;
      candidate.owner_pose = pose.position;
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

    // A route that touches a sensing pose already used is a walk back down
    // the alley, even when the final point is new. Drop it. If nothing
    // ahead remains, retreat to the entry.
    const auto& driven = observation.entry_path;
    const mars::common::Point2D here = observation.pose;
    observation.routes.erase(
        std::remove_if(
            observation.routes.begin(), observation.routes.end(),
            [&](const mars::navigation_decision::ReachableRoute& route) {
              const auto stepped =
                  mars::navigation_decision::cap_last_hop(route.skeleton,
                                                          config.max_step);
              if (stepped.size() < 2) {
                return false;
              }
              for (const auto& visited : driven) {
                if (distance(visited, here) <= config.linear_epsilon) {
                  continue;
                }
                for (std::size_t vertex = 1; vertex < stepped.size();
                     ++vertex) {
                  if (distance(stepped[vertex], visited) <=
                      config.arrival_tolerance) {
                    return true;
                  }
                }
              }
              return false;
            }),
        observation.routes.end());
    if (observation.routes.empty()) {
      observation.ranked.clear();
      observation.escape_candidates.clear();
    }
    if (const auto portals = manager.return_gates(); portals.success) {
      observation.gates = portals.gates;
    }

    auto decision = navigator.decide(observation);
    if (decision.event == "FUNNEL" &&
        path_crosses_wall(decision.planned_path.points, map) &&
        entry_path.size() >= 2) {
      decision.planned_path.points.assign(entry_path.rbegin(), entry_path.rend());
      decision.planned_path.length = 0.0;
      for (std::size_t index = 1; index < decision.planned_path.points.size();
           ++index) {
        decision.planned_path.length += distance(
            decision.planned_path.points[index - 1],
            decision.planned_path.points[index]);
      }
    }
    mission.frames.push_back(make_frame(mission.frames.size(), pose,
                                        config.vision_radius, perception,
                                        manager, decision));
    mission.frames.back().trail_index =
        mission.trail.empty() ? 0 : mission.trail.size() - 1;

    if (decision.state == mars::navigation_decision::NavigationState::GoalReached ||
        decision.state == mars::navigation_decision::NavigationState::Failed) {
      mission.stop_reason = decision.event;
      return mission;
    }
    if (decision.event == "RETURN_REACHED") {
      mission.saw_return = true;
      mission.stop_reason = "RETURN_REACHED";
      if (distance(pose.position, scenario.start) <= config.arrival_tolerance) {
        return mission;
      }
      stalls = 0;
      continue;
    }

    follower.set_path(decision.planned_path.points);
    if (decision.planned_path.points.size() < 2 || follower.path_done()) {
      ++stalls;
      if (stalls >= limits.stall_limit) {
        mission.follower_stuck = true;
        mission.stop_reason = "STALL";
        return mission;
      }
      continue;
    }

    if (decision.event == "FUNNEL" &&
        mission.return_trail_index >= mission.trail.size() &&
        !mission.trail.empty()) {
      mission.return_trail_index = mission.trail.size() - 1;
    }

    int zero_commands = 0;
    double travel = 0.0;
    for (int sample = 0; sample < limits.max_follower_steps && !follower.path_done();
         ++sample) {
      const BodyTwist command = follower.compute(odom.pose());
      if (follower.path_done()) {
        break;
      }
      if (std::abs(command.linear_x) < 1.0e-9 &&
          std::abs(command.angular_z) < 1.0e-9) {
        ++zero_commands;
        if (zero_commands >= 5) {
          mission.follower_stuck = true;
          mission.stop_reason = "STALL";
          return mission;
        }
        continue;
      }
      zero_commands = 0;
      const mars::common::Point2D step_from = odom.pose().position;
      odom.integrate(command, limits.dt);
      travel += distance(step_from, odom.pose().position);
      mission.trail.push_back(odom.pose().position);
    }

    if (decision.event == "EXPLORE" || decision.event == "GOAL_VISIBLE") {
      mission.entry_length += travel;
      if (travel > 0.01) {
        mission.saw_entry = true;
      }
      entry_path.push_back(odom.pose().position);
    } else if (decision.event == "FUNNEL") {
      mission.return_length += travel;
      if (travel > 0.01) {
        mission.saw_return = true;
      }
    }
    if (travel < 0.01) {
      ++stalls;
      if (stalls >= limits.stall_limit) {
        mission.follower_stuck = true;
        mission.stop_reason = "STALL";
        return mission;
      }
    } else {
      stalls = 0;
    }
  }

  mission.stop_reason = "MAX_STEPS";
  return mission;
}

}  // namespace mars::demo_simulation
