#include "mars/graph_bundle_management/graph_bundle_manager.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "internal/geometry_helpers.hpp"
#include "mars/graph_bundle_management/bundle_builder.hpp"
#include "mars/graph_bundle_management/gate_preparation.hpp"
#include "mars/graph_bundle_management/graph_search.hpp"
#include "mars/graph_bundle_management/sequence_extractor.hpp"
#include "mars/graph_bundle_management/shared_sight_connector.hpp"
#include "mars/perception_geometry/geometry.hpp"

namespace mars::graph_bundle_management {
namespace {

void validate_config(const GraphBundleConfig& config) {
  const auto valid_non_negative = [](double value) {
    return std::isfinite(value) && value >= 0.0;
  };
  if (!valid_non_negative(config.rank_alpha) ||
      !valid_non_negative(config.rank_beta) ||
      !valid_non_negative(config.linear_epsilon) ||
      !valid_non_negative(config.angular_epsilon) ||
      !valid_non_negative(config.open_point_merge_radius) ||
      !valid_non_negative(config.visited_radius)) {
    throw std::invalid_argument(
        "all graph-bundle configuration scalars must be finite and non-negative");
  }
}

}  // namespace

GraphBundleManager::GraphBundleManager(GraphBundleConfig config)
    : config_(config), open_point_memory_(config.open_point_merge_radius) {
  validate_config(config_);
}

UpdateSummary GraphBundleManager::update(const GraphBundleUpdate& input) {
  const auto& sight = input.perception.neighbor_sight;
  internal::require_finite(input.current_pose.position, "current pose position");
  internal::require_finite(input.goal, "goal");
  internal::require_finite(sight.center, "neighbor sight center");
  if (!std::isfinite(input.current_pose.yaw)) {
    throw std::invalid_argument("current pose yaw must be finite");
  }
  mars::perception_geometry::validate_circle({sight.center, sight.radius});
  if (!internal::point_near(input.current_pose.position, sight.center,
                            config_.linear_epsilon)) {
    throw std::invalid_argument(
        "neighbor sight center does not agree with current pose");
  }
  if (sightings_.count(input.observation_id) != 0U) {
    throw std::invalid_argument("duplicate observation ID");
  }

  for (const auto& boundary : sight.visible_boundaries) {
    internal::require_finite(boundary.start, "visible boundary start");
    internal::require_finite(boundary.end, "visible boundary end");
    if (internal::distance(sight.center, boundary.start) >
            sight.radius + config_.linear_epsilon ||
        internal::distance(sight.center, boundary.end) >
            sight.radius + config_.linear_epsilon) {
      throw std::invalid_argument(
          "visible boundary endpoint is outside the Module 1 sensing circle");
    }
  }
  for (const auto& open_sight : input.perception.open_sights) {
    mars::perception_geometry::validate_interval(open_sight.interval);
    if (open_sight.interval.sweep <=
        mars::perception_geometry::angular_epsilon) {
      throw std::invalid_argument(
          "Module 1 open sights must have positive angular measure");
    }
  }

  // Validate all geometry before changing accumulated state. The temporary
  // center ID is replaced after the graph node receives its stable ID.
  auto bundle = build_bundle(input.perception, input.observation_id, 0,
                             config_.linear_epsilon);
  for (const auto& open_point : input.perception.open_points) {
    internal::require_finite(open_point.point, "open point");
    if (!std::isfinite(open_point.angle) || open_point.angle < 0.0 ||
        open_point.angle >= mars::common::two_pi) {
      throw std::invalid_argument(
          "open point angle must use Module 1's normalized [0, 2*pi) convention");
    }
    if (open_point.sight_index &&
        *open_point.sight_index >= input.perception.open_sights.size()) {
      throw std::invalid_argument("open point sight_index is out of range");
    }
    if (open_point.sight_index) {
      const double midpoint = mars::perception_geometry::angular_midpoint(
          input.perception.open_sights[*open_point.sight_index].interval);
      if (std::abs(std::remainder(open_point.angle - midpoint,
                                  mars::common::two_pi)) >
          config_.angular_epsilon) {
        throw std::invalid_argument(
            "open point angle does not match its source open sight");
      }
    }
    if (!neighbor_sight_supports_point(sight, open_point.point,
                                       config_.linear_epsilon)) {
      throw std::invalid_argument(
          "open point is not supported by the current neighbor sight");
    }
    const double radial_distance =
        internal::distance(sight.center, open_point.point);
    if (std::abs(radial_distance - sight.radius) > config_.linear_epsilon) {
      throw std::invalid_argument(
          "open point is not on the Module 1 neighbor-sight circle");
    }
    const double radial_angle = mars::perception_geometry::normalize_angle(
        std::atan2(open_point.point.y - sight.center.y,
                   open_point.point.x - sight.center.x));
    if (std::abs(std::remainder(open_point.angle - radial_angle,
                                mars::common::two_pi)) >
        config_.angular_epsilon) {
      throw std::invalid_argument(
          "open point angle does not match its Module 1 point geometry");
    }
  }

  const auto observed_ids =
      open_point_memory_.ingest(input.perception, input.observation_id);
  open_point_memory_.mark_within_radius(input.current_pose.position,
                                        config_.visited_radius,
                                        OpenPointStatus::Reached);

  const auto center_node_id = visibility_graph_.add_or_merge_node(
      input.current_pose.position, VisibilityNodeKind::ObservationCenter,
      input.observation_id, std::nullopt, config_.linear_epsilon);

  for (const auto open_point_id : observed_ids) {
    const auto* record = open_point_memory_.find(open_point_id);
    if (record == nullptr || record->status != OpenPointStatus::Active) {
      continue;
    }
    const auto open_node_id = visibility_graph_.add_or_merge_node(
        record->point, VisibilityNodeKind::OpenPoint, input.observation_id,
        record->id, config_.open_point_merge_radius);
    visibility_graph_.add_visible_edge(
        center_node_id, open_node_id, VisibilityEvidence::NeighborSight,
        {input.observation_id});
  }

  for (const auto& [previous_observation_id, previous_sight] : sightings_) {
    if (!neighbor_sights_prove_connection(previous_sight, sight,
                                           config_.linear_epsilon)) {
      continue;
    }
    const auto previous_center =
        visibility_graph_.node_for_observation(previous_observation_id);
    if (previous_center) {
      visibility_graph_.add_visible_edge(
          *previous_center, center_node_id,
          VisibilityEvidence::SharedSightOverlap,
          {previous_observation_id, input.observation_id});
    }
  }

  bundle.center_node_id = center_node_id;
  const bool bundle_degenerate = bundle.degenerate;
  bundle_history_.add(std::move(bundle));
  sightings_.emplace(input.observation_id, sight);
  current_center_node_id_ = center_node_id;
  ranked_open_points_ = rank_open_points(
      open_point_memory_.records(), input.current_pose.position, input.goal,
      config_.rank_alpha, config_.rank_beta, config_.linear_epsilon,
      config_.angular_epsilon);

  return {input.observation_id,
          center_node_id,
          observed_ids,
          ranked_open_points_.size(),
          visibility_graph_.nodes().size(),
          visibility_graph_.edges().size(),
          bundle_degenerate};
}

std::vector<RankedOpenPoint> GraphBundleManager::ranked_open_points() const {
  return ranked_open_points_;
}

const VisibilityGraph& GraphBundleManager::visibility_graph() const noexcept {
  return visibility_graph_;
}

const OpenPointMemory& GraphBundleManager::open_point_memory() const noexcept {
  return open_point_memory_;
}

const BundleHistory& GraphBundleManager::bundle_history() const noexcept {
  return bundle_history_;
}

RoutePreparationResult GraphBundleManager::prepare_route(
    OpenPointId target) const {
  return prepare_route(target, config_.search_policy);
}

RoutePreparationResult GraphBundleManager::prepare_route(
    OpenPointId target, GraphSearchPolicy policy) const {
  const auto* record = open_point_memory_.find(target);
  if (record == nullptr) {
    return {false, RouteFailureReason::UnknownTarget,
            "The requested open-point ID is unknown.", {}, {}, {}};
  }
  if (record->status != OpenPointStatus::Active) {
    return {false, RouteFailureReason::TargetUnavailable,
            "The requested open point is not active.", {}, {}, {}};
  }
  if (!current_center_node_id_) {
    return {false, RouteFailureReason::NoCurrentObservation,
            "No observation has been stored yet.", {}, {}, {}};
  }
  const auto target_node = visibility_graph_.node_for_open_point(target);
  if (!target_node) {
    return {false, RouteFailureReason::TargetNodeMissing,
            "The target has no visibility-graph node.", {}, {}, {}};
  }

  const auto path = policy == GraphSearchPolicy::BreadthFirst
                        ? breadth_first_path(visibility_graph_,
                                             *current_center_node_id_, *target_node)
                        : shortest_cost_path(visibility_graph_,
                                             *current_center_node_id_, *target_node);
  if (!path) {
    return {false, RouteFailureReason::NoGraphPath,
            "No visibility-proven graph path reaches the target.", {}, {}, {}};
  }
  auto sequence = extract_bundle_sequence(
      *path, visibility_graph_, bundle_history_,
      BundleSequenceDirection::CurrentToTarget);
  if (!sequence.success) {
    return {false, RouteFailureReason::BundleSequenceFailure,
            sequence.message, *path, {}, {}};
  }
  auto gates = prepare_gates(sequence.sequence, config_.gate_contract_enabled,
                             config_.linear_epsilon);
  return {true, RouteFailureReason::None, {}, *path,
          std::move(sequence.sequence), std::move(gates)};
}

GatePreparationResult GraphBundleManager::return_gates() const {
  BundleSequence sequence;
  sequence.direction = BundleSequenceDirection::TargetToCurrent;
  const auto& stored = bundle_history_.bundles();
  sequence.bundles.reserve(stored.size());
  for (auto bundle = stored.rbegin(); bundle != stored.rend(); ++bundle) {
    sequence.observation_ids.push_back(bundle->observation_id);
    sequence.bundles.push_back(*bundle);
  }
  return prepare_gates(sequence, config_.gate_contract_enabled,
                       config_.linear_epsilon);
}

}  // namespace mars::graph_bundle_management
