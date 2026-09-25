#include <exception>
#include <iostream>
#include <string>

#include "mode1_mission.hpp"
#include "mode1_trace.hpp"

#ifndef MARS_MODE1_MAP_DIR
#define MARS_MODE1_MAP_DIR "maps"
#endif

namespace {

void print_usage() {
  std::cerr << "Usage: polygon_explore_sim [--map hard_alley|geogebra]\n"
            << "       polygon_explore_sim --list\n"
            << "Default map is hard_alley. The robot follows each planned\n"
            << "path with fake odometry, then writes mode1_<map>.svg.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_id = "hard_alley";
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--list") {
      for (const auto& id : mars::demo_simulation::mode1_map_ids()) {
        const auto& scenario = mars::demo_simulation::mode1_scenario(id);
        std::cout << id << "  " << scenario.occupancy_yaml << "  start=("
                  << scenario.start.x << ", " << scenario.start.y << ")  goal=("
                  << scenario.goal.x << ", " << scenario.goal.y << ")\n";
      }
      return 0;
    }
    if (arg == "--help" || arg == "-h") {
      print_usage();
      return 0;
    }
    if (arg == "--map") {
      if (i + 1 >= argc) {
        print_usage();
        return 2;
      }
      map_id = argv[++i];
      continue;
    }
    print_usage();
    return 2;
  }

  try {
    const auto& scenario = mars::demo_simulation::mode1_scenario(map_id);
    const std::string yaml =
        std::string(MARS_MODE1_MAP_DIR) + "/" + scenario.occupancy_yaml;
    const auto map = mars::demo_simulation::load_occupancy_map(yaml);
    const auto mission = mars::demo_simulation::run_mode1_mission(scenario, map);
    const std::string svg = "mode1_" + scenario.id + ".svg";
    mars::demo_simulation::write_marker_svg(svg, map, mission);
    std::cout << "Mode 1 map: " << scenario.id << "\n"
              << "occupancy: " << map.yaml_path << "  obstacles="
              << map.obstacles.size() << "\n"
              << mars::demo_simulation::format_marker_log(mission)
              << "trace: " << svg << "\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
