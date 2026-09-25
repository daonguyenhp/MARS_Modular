#pragma once

#include <string>
#include <vector>

#include "mars/common/geometry_types.hpp"
#include "mode1_map.hpp"

namespace mars::demo_simulation {

struct AngleSpan {
  double start{0.0};
  double sweep{0.0};
};

struct MarkerSegment {
  mars::common::Point2D first{};
  mars::common::Point2D second{};
};

/** One sensing pose, the layers MD4.3 has to show. */
struct MarkerFrame {
  std::size_t index{0};
  mars::common::Point2D pose{};
  double yaw{0.0};
  double vision_radius{0.0};
  std::vector<AngleSpan> closed{};
  std::vector<AngleSpan> open{};
  std::vector<mars::common::Point2D> open_points{};
  std::size_t graph_nodes{0};
  std::size_t graph_edges{0};
  std::vector<MarkerSegment> graph{};
  std::vector<mars::common::Point2D> planned_path{};
  std::size_t bundles{0};
  std::vector<mars::common::Point2D> bundle_centers{};
  std::vector<mars::common::Point2D> funnel{};
  std::string event{};
  /** Index into MissionResult::trail at this sensing pose. */
  std::size_t trail_index{0};
};

struct MissionConfig {
  int max_observations{80};
  int max_follower_steps{2000};
  int stall_limit{20};
  double dt{0.1};
};

struct MissionResult {
  std::string stop_reason{};
  bool follower_stuck{false};
  bool saw_entry{false};
  bool saw_return{false};
  double entry_length{0.0};
  double return_length{0.0};
  std::vector<MarkerFrame> frames{};
  std::vector<mars::common::Point2D> trail{};
  /** First trail index driven on the return. trail.size() means there is no return yet. */
  std::size_t return_trail_index{static_cast<std::size_t>(-1)};
};

MissionResult run_mode1_mission(const Mode1Scenario& scenario,
                                const LoadedMap& map,
                                MissionConfig config = {});

}  // namespace mars::demo_simulation
