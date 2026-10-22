#pragma once

#include <string>

#include "mode1_map.hpp"
#include "mode1_mission.hpp"

namespace mars::demo_simulation {

std::string format_marker_log(const MissionResult& mission);
void write_marker_svg(const std::string& path, const LoadedMap& map,
                      const MissionResult& mission);

/** Four pictures of one alley: sights, graph, bundles, exit. */
void write_algorithm_panels(const std::string& directory, const LoadedMap& map,
                            const MissionResult& mission,
                            mars::common::Point2D goal);

}  // namespace mars::demo_simulation
