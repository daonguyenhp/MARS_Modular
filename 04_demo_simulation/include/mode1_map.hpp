#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "mars/common/geometry_types.hpp"

namespace mars::demo_simulation {

/**
 * Launch choice for one Mode 1 run.
 * Start and goal are the polygon_explore_sim.launch arguments.
 * Obstacle geometry is not stored here; load_occupancy_map reads it.
 */
struct Mode1Scenario {
  std::string id{};
  std::string occupancy_yaml{};
  mars::common::Point2D start{};
  mars::common::Point2D goal{};
};

/** Obstacles traced from a map_server yaml + pgm. */
struct LoadedMap {
  std::string yaml_path{};
  double resolution{0.0};
  mars::common::Point2D origin{};
  int width{0};
  int height{0};
  std::vector<mars::common::Polygon2D> obstacles{};
  /** Row 0 is the top of the PGM. 0 is free, 100 is occupied. */
  std::vector<std::int8_t> occupancy{};
};

std::vector<std::string> mode1_map_ids();

/** Throws std::invalid_argument when id is not hard_alley or geogebra. */
const Mode1Scenario& mode1_scenario(std::string_view id);

/** Throws std::runtime_error when the yaml or pgm cannot be read. */
LoadedMap load_occupancy_map(const std::string& yaml_path);

}  // namespace mars::demo_simulation
