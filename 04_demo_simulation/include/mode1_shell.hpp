#pragma once

#include <cstddef>
#include <string>

#include "mode1_map.hpp"
#include "navigator.hpp"

namespace mars::demo_simulation {

/**
 * One observation at the scenario start.
 * Module 1 perceives LoadedMap::obstacles, Module 2 ranks them,
 * Module 3 decides. MD4.2 follows the path with fake odom.
 * MD4.3 records the marker layers for that sensing pose.
 */
struct Mode1Tick {
  std::size_t wall_count{0};
  std::size_t closed_sights{0};
  std::size_t open_sights{0};
  std::size_t open_points{0};
  std::size_t active_open_points{0};
  mars::navigation_decision::NavigationState state{
      mars::navigation_decision::NavigationState::Explore};
  std::string event{};
  std::string log{};
};

Mode1Tick run_mode1_tick(const Mode1Scenario& scenario, const LoadedMap& map);

}  // namespace mars::demo_simulation
