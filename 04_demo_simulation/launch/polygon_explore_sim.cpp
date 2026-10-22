#include <exception>
#include <filesystem>
#include <iostream>
#include <string>

#include "mode1_mission.hpp"
#include "mode1_trace.hpp"

#ifndef MARS_MODE1_MAP_DIR
#define MARS_MODE1_MAP_DIR "maps"
#endif
#ifndef MARS_MODE1_OUTPUT_DIR
#define MARS_MODE1_OUTPUT_DIR "04_demo_simulation/output"
#endif

namespace {

void print_usage() {
  std::cerr << "Usage: polygon_explore_sim [--map hard_alley|geogebra|fig8]\n"
            << "       polygon_explore_sim --list\n"
            << "Default map is hard_alley. fig8 is the paper maze and writes\n"
            << "four pictures, one layer each, under output/.\n";
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
    if (map_id == "fig8") {
      const auto scenario = mars::demo_simulation::paper_fig8_scenario();
      const auto map = mars::demo_simulation::paper_fig8_map();
      mars::demo_simulation::MissionConfig limits;
      limits.vision_radius = 16.0;
      limits.max_step = 20.0;
      limits.follower_speed = 6.0;
      limits.max_observations = 80;
      limits.max_follower_steps = 4000;
      const auto mission =
          mars::demo_simulation::run_mode1_mission(scenario, map, limits);
      std::filesystem::create_directories(MARS_MODE1_OUTPUT_DIR);
      mars::demo_simulation::write_algorithm_panels(MARS_MODE1_OUTPUT_DIR, map,
                                                    mission, scenario.goal);
      std::cout << "Mode 1 map: fig8\n"
                << "obstacles=" << map.obstacles.size() << "\n"
                << mars::demo_simulation::format_marker_log(mission)
                << "panels: " << MARS_MODE1_OUTPUT_DIR
                << "/fig8a_sights.svg fig8b_graph.svg fig9b_bundles.svg "
                   "fig9c_exit.svg\n";
      return mission.stop_reason == "STALL" ? 1 : 0;
    }
    const auto& scenario = mars::demo_simulation::mode1_scenario(map_id);
    const std::string yaml =
        std::string(MARS_MODE1_MAP_DIR) + "/" + scenario.occupancy_yaml;
    const auto map = mars::demo_simulation::load_occupancy_map(yaml);
    const auto mission = mars::demo_simulation::run_mode1_mission(scenario, map);
    const std::string svg_name = "mode1_" + scenario.id + ".svg";
    mars::demo_simulation::write_marker_svg(svg_name, map, mission);
    std::filesystem::create_directories(MARS_MODE1_OUTPUT_DIR);
    const std::filesystem::path packaged =
        std::filesystem::path(MARS_MODE1_OUTPUT_DIR) / svg_name;
    mars::demo_simulation::write_marker_svg(packaged.string(), map, mission);
    std::cout << "Mode 1 map: " << scenario.id << "\n"
              << "occupancy: " << map.yaml_path << "  obstacles="
              << map.obstacles.size() << "\n"
              << mars::demo_simulation::format_marker_log(mission)
              << "trace: " << svg_name << "\n"
              << "packaged: " << packaged.string() << "\n";
  } catch (const std::exception& error) {
    std::cerr << error.what() << "\n";
    return 1;
  }
  return 0;
}
