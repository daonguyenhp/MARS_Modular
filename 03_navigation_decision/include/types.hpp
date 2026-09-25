#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mars/common/geometry_types.hpp"
#include "mars/graph_bundle_management/types.hpp"

namespace mars::navigation_decision {

using mars::common::Point2D;
using mars::graph_bundle_management::Gate;
using mars::graph_bundle_management::OpenPointId;
using mars::graph_bundle_management::RankedOpenPoint;

/** Algorithm 2 modes. FAILED is frontier exhausted at the entry. */
enum class NavigationState {
  Explore,
  Escape,
  GoalReached,
  Failed,
};

enum class ExploreKind {
  GoalReached,
  GoalVisible,
  Step,
  BlindAlley,
};

enum class EscapeKind {
  Retreat,
  FrontierExhausted,
};

enum class FunnelFailureReason {
  None,
  EmptyPath,
  DegenerateGate,
};

/**
 * JetTank / paper limited-vision numbers.
 *
 * vision_radius r: Mode 1 default 0.85 m (YDLIDAR G4 clipped to paper r).
 * max_step must stay < 2 r so consecutive concurrent points have overlapping
 * neighbor sights (paper Section 4.1). Mode 1 uses 0.70 m.
 * goal_tolerance must be >= the path-follower waypoint tolerance (0.10 m);
 * Mode 1 uses 0.15 m.
 */
struct NavigationConfig {
  double vision_radius{0.85};
  double goal_tolerance{0.15};
  double max_step{0.70};
  double arrival_tolerance{0.15};
  double linear_epsilon{1.0e-9};
};

/** Module 2 skeleton already converted to map-frame metres. */
struct ReachableRoute {
  OpenPointId id{0};
  std::vector<Point2D> skeleton{};
};

/**
 * Remaining global open point with the concurrent point that owns it.
 * owner_pose is OpenPointRecord::source_center from Module 2.
 */
struct EscapeCandidate {
  OpenPointId id{0};
  Point2D point{};
  Point2D owner_pose{};
  double score{0.0};
};

struct PlannedPath {
  std::vector<Point2D> points{};
  double length{0.0};
};

struct ExploreResult {
  ExploreKind kind{ExploreKind::BlindAlley};
  std::string reason{};
  std::optional<OpenPointId> target_id{};
  std::optional<Point2D> next_point{};
  PlannedPath path{};
};

struct EscapePlan {
  EscapeKind kind{EscapeKind::FrontierExhausted};
  std::string reason{};
  std::optional<OpenPointId> retired_id{};
  std::optional<Point2D> target{};
  std::string log{};
};

struct FunnelResult {
  bool success{false};
  FunnelFailureReason failure_reason{FunnelFailureReason::None};
  std::string message{};
  PlannedPath path{};
};

struct NavigationObservation {
  Point2D pose{};
  Point2D goal{};
  bool goal_visible{false};
  std::vector<RankedOpenPoint> ranked{};
  std::vector<ReachableRoute> routes{};
  std::vector<EscapeCandidate> escape_candidates{};
  Point2D entry_pose{};
  std::vector<Point2D> entry_path{};
  std::vector<Gate> gates{};
  bool at_return_target{false};
};

struct NavigationDecision {
  NavigationState state{NavigationState::Explore};
  std::string event{};
  std::string log{};
  std::optional<OpenPointId> selected_id{};
  std::optional<Point2D> selected_target{};
  PlannedPath planned_path{};
  std::optional<bool> return_leq_entry{};
};

double path_length(const std::vector<Point2D>& points);
std::vector<Point2D> cap_last_hop(std::vector<Point2D> path, double max_step);
void validate_config(const NavigationConfig& config);

}  // namespace mars::navigation_decision
