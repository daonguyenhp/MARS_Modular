#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

#include "mode1_mission.hpp"
#include "mode1_shell.hpp"
#include "mode1_trace.hpp"
#include "path_follower.hpp"

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

bool near(double value, double expected) {
  return std::abs(value - expected) < 1.0e-9;
}

std::string read_file(const std::string& path) {
  std::ifstream input(path);
  require(static_cast<bool>(input), "cannot open " + path);
  std::ostringstream buffer;
  buffer << input.rdbuf();
  return buffer.str();
}

mars::demo_simulation::LoadedMap load_named(const std::string& id) {
  const auto& scenario = mars::demo_simulation::mode1_scenario(id);
  return mars::demo_simulation::load_occupancy_map(
      std::string(MARS_MODE1_MAP_DIR) + "/" + scenario.occupancy_yaml);
}

void test_catalog() {
  const auto ids = mars::demo_simulation::mode1_map_ids();
  require(ids.size() == 2, "Mode 1 should expose hard_alley and geogebra");

  const auto& hard = mars::demo_simulation::mode1_scenario("hard_alley");
  require(near(hard.start.x, 0.35) && near(hard.start.y, 0.35),
          "hard_alley start");
  require(near(hard.goal.x, 0.35) && near(hard.goal.y, 3.40),
          "hard_alley goal");

  const auto& geo = mars::demo_simulation::mode1_scenario("geogebra");
  require(near(geo.start.x, 0.35) && near(geo.start.y, 0.35), "geogebra start");
  require(near(geo.goal.x, 0.35) && near(geo.goal.y, 2.90), "geogebra goal");

  bool threw = false;
  try {
    mars::demo_simulation::mode1_scenario("big_alley");
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  require(threw, "unknown map should be rejected");
}

void test_loaded_map(const std::string& id, int width, int height,
                     const std::string& origin) {
  const auto& scenario = mars::demo_simulation::mode1_scenario(id);
  const std::string yaml_path =
      std::string(MARS_MODE1_MAP_DIR) + "/" + scenario.occupancy_yaml;
  const std::string yaml = read_file(yaml_path);
  require(yaml.find("resolution: 0.025") != std::string::npos,
          id + " yaml resolution drifted");
  require(yaml.find(origin) != std::string::npos, id + " yaml origin drifted");

  const auto map = mars::demo_simulation::load_occupancy_map(yaml_path);
  require(near(map.resolution, 0.025), id + " loader resolution");
  require(map.width == width && map.height == height, id + " pgm size");
  require(!map.obstacles.empty(), id + " produced no obstacles from the pgm");
  require(yaml.find("image:") != std::string::npos, id + " yaml has no image");
}

void test_first_tick(const std::string& id) {
  const auto& scenario = mars::demo_simulation::mode1_scenario(id);
  const auto map = load_named(id);
  const auto tick = mars::demo_simulation::run_mode1_tick(scenario, map);
  require(tick.wall_count == map.obstacles.size(),
          id + " tick did not use the loaded obstacles");
  require(tick.closed_sights + tick.open_sights > 0,
          id + " start pose sees no sights");
  require(tick.open_points > 0, id + " start pose has no open points");
  require(!tick.event.empty(), id + " produced no decision event");
  std::cout << id << " obstacles=" << map.obstacles.size()
            << " event=" << tick.event
            << " open_points=" << tick.open_points << "\n";
}

void test_follower_reaches_path_end() {
  mars::demo_simulation::FollowerConfig follower;
  mars::navigation_decision::NavigationConfig planner;
  require(planner.goal_tolerance >= follower.waypoint_tolerance,
          "planner tolerance must cover the follower waypoint band");

  mars::demo_simulation::PathFollower short_hop(follower);
  short_hop.set_path({{0.12, 0.0}});
  mars::demo_simulation::FakeOdom parked{{{0.0, 0.0}, 0.0}};
  const auto first = short_hop.compute(parked.pose());
  require(!short_hop.path_done(), "0.12 m is still outside the stop band");
  require(first.linear_x > 0.0, "follower should drive toward the waypoint");

  mars::demo_simulation::PathFollower tracker(follower);
  tracker.set_path({{0.0, 0.0}, {0.50, 0.0}});
  mars::demo_simulation::FakeOdom odom{{{0.0, 0.0}, 0.0}};
  for (int step = 0; step < 2000 && !tracker.path_done(); ++step) {
    const auto command = tracker.compute(odom.pose());
    if (tracker.path_done()) {
      break;
    }
    require(std::abs(command.linear_x) > 0.0 || std::abs(command.angular_z) > 0.0,
            "follower stopped short of the path");
    odom.integrate(command, 0.1);
  }
  require(tracker.path_done(), "follower did not finish the path");
  const double leftover =
      std::hypot(odom.pose().position.x - 0.50, odom.pose().position.y);
  require(leftover < follower.goal_tolerance,
          "robot stopped outside the follower goal tolerance");
}

void test_marker_log_names_the_layers() {
  mars::demo_simulation::MissionConfig limits;
  limits.max_observations = 2;
  limits.max_follower_steps = 40;
  const auto& scenario = mars::demo_simulation::mode1_scenario("hard_alley");
  const auto map = load_named("hard_alley");
  const auto mission =
      mars::demo_simulation::run_mode1_mission(scenario, map, limits);
  const std::string log = mars::demo_simulation::format_marker_log(mission);
  require(log.find("marker vision_circle") != std::string::npos, "vision circle");
  require(log.find("marker closed") != std::string::npos, "closed sights");
  require(log.find("marker open ") != std::string::npos, "open sights");
  require(log.find("marker open_points") != std::string::npos, "open points");
  require(log.find("marker graph") != std::string::npos, "visibility graph");
  require(log.find("marker bundles") != std::string::npos, "bundle sequence");
  require(log.find("marker funnel") != std::string::npos, "funnel path");
  require(!mission.follower_stuck, "follower stalled on the opening hops");
  require(!mission.frames.empty(), "mission recorded no sensing frames");
}

}  // namespace

int main() {
  try {
    test_catalog();
    test_loaded_map("hard_alley", 203, 218, "origin: [-1.5, -1.5, 0.0]");
    test_loaded_map("geogebra", 180, 200, "origin: [-1.25, -1.25, 0.0]");
    test_first_tick("hard_alley");
    test_first_tick("geogebra");
    test_follower_reaches_path_end();
    test_marker_log_names_the_layers();
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
