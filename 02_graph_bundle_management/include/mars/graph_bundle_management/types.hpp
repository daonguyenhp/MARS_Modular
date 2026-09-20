#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mars/common/geometry_types.hpp"
#include "mars/perception_geometry/types.hpp"

namespace mars::graph_bundle_management {

using OpenPointId = std::uint64_t;
using ObservationId = std::uint64_t;
using VisibilityNodeId = std::uint64_t;

/** Lifecycle state kept independently from the current ranking. */
enum class OpenPointStatus {
  Active,
  Selected,
  Reached,
  InactiveExplored,
  Invalid,
};

/** Persistent metadata for one geometrically merged exploration target. */
struct OpenPointRecord {
  OpenPointId id{0};
  mars::common::Point2D point{};
  double source_angle{0.0};
  std::optional<std::size_t> source_sight_index{};
  ObservationId first_observation_id{0};
  ObservationId latest_observation_id{0};
  mars::common::Point2D source_center{};
  OpenPointStatus status{OpenPointStatus::Active};
};

/** A current-pose-dependent view of an active open point. */
struct RankedOpenPoint {
  OpenPointId id{0};
  mars::common::Point2D point{};
  double distance_to_goal{0.0};
  double angle_to_goal{0.0};
  double score{0.0};
  ObservationId discovery_observation_id{0};
};

/** Semantic role of a node in the accumulated visibility graph. */
enum class VisibilityNodeKind {
  ObservationCenter,
  OpenPoint,
  SharedSightAnchor,
};

/** Stable graph node with optional source metadata. */
struct VisibilityNode {
  VisibilityNodeId id{0};
  mars::common::Point2D position{};
  VisibilityNodeKind kind{VisibilityNodeKind::ObservationCenter};
  std::optional<ObservationId> observation_id{};
  std::optional<OpenPointId> open_point_id{};
};

/** Geometric evidence used to admit an edge. */
enum class VisibilityEvidence {
  NeighborSight,
  SharedSightOverlap,
};

/** Undirected visibility edge with Euclidean traversal cost. */
struct VisibilityEdge {
  VisibilityNodeId first{0};
  VisibilityNodeId second{0};
  double cost{0.0};
  VisibilityEvidence evidence{VisibilityEvidence::NeighborSight};
  std::vector<ObservationId> supporting_observations{};
};

/** Ordered graph node path and its accumulated edge cost. */
struct GraphPath {
  std::vector<VisibilityNodeId> node_ids{};
  double total_cost{0.0};
};

/** Explicit route-search objective. */
enum class GraphSearchPolicy {
  BreadthFirst,
  ShortestCost,
};

/** Paper bundle: a fan from one concurrent point to visible vertices. */
struct Bundle {
  ObservationId observation_id{0};
  VisibilityNodeId center_node_id{0};
  mars::common::Point2D concurrent_point{};
  std::vector<mars::common::Point2D> ordered_vertices{};
  std::vector<mars::common::Segment2D> segments{};
  bool degenerate{true};
};

/** Direction in which a graph path is converted to bundles. */
enum class BundleSequenceDirection {
  CurrentToTarget,
  TargetToCurrent,
};

/** Normal failure modes for route-to-bundle resolution. */
enum class SequenceFailureReason {
  None,
  EmptyGraphPath,
  MissingBundle,
  DegenerateBundle,
};

/** Bundles and their observations in the requested skeleton travel order. */
struct BundleSequence {
  std::vector<VisibilityNodeId> skeleton_node_ids{};
  std::vector<ObservationId> observation_ids{};
  std::vector<Bundle> bundles{};
  BundleSequenceDirection direction{BundleSequenceDirection::CurrentToTarget};
};

/** Typed outcome of bundle-sequence extraction. */
struct BundleSequenceResult {
  bool success{false};
  SequenceFailureReason failure_reason{SequenceFailureReason::None};
  std::string message{};
  BundleSequence sequence{};
};

/** Endpoint ordering used by a future Module 3 portal contract. */
enum class GateOrientation {
  LeftToRight,
  RightToLeft,
};

/** A validated portal and the bundle metadata that supports it. */
struct Gate {
  mars::common::Point2D left{};
  mars::common::Point2D right{};
  ObservationId source_observation_id{0};
  std::optional<ObservationId> paired_observation_id{};
  std::size_t sequence_index{0};
  GateOrientation orientation{GateOrientation::LeftToRight};
  bool valid{false};
};

/** Normal failure modes for gate preparation. */
enum class GateFailureReason {
  None,
  ContractNotEnabled,
  EmptySequence,
  UnsupportedGeometry,
  InvalidGate,
};

/** Typed outcome of gate preparation. */
struct GatePreparationResult {
  bool success{false};
  GateFailureReason failure_reason{GateFailureReason::None};
  std::string message{};
  std::vector<Gate> gates{};
};

/** Complete immutable input required for one Module 2 update. */
struct GraphBundleUpdate {
  ObservationId observation_id{0};
  mars::common::Pose2D current_pose{};
  mars::common::Point2D goal{};
  mars::perception_geometry::PerceptionResult perception{};
};

/** All numeric policies used by Module 2. */
struct GraphBundleConfig {
  double rank_alpha{1.0};
  double rank_beta{1.0};
  double linear_epsilon{1.0e-9};
  double angular_epsilon{1.0e-9};
  double open_point_merge_radius{0.05};
  double visited_radius{0.05};
  GraphSearchPolicy search_policy{GraphSearchPolicy::ShortestCost};
  bool gate_contract_enabled{false};
};

/** Immutable diagnostics returned after accepting an observation. */
struct UpdateSummary {
  ObservationId observation_id{0};
  VisibilityNodeId center_node_id{0};
  std::vector<OpenPointId> observed_open_point_ids{};
  std::size_t active_open_point_count{0};
  std::size_t visibility_node_count{0};
  std::size_t visibility_edge_count{0};
  bool bundle_degenerate{true};
};

/** Normal failure modes for historical route preparation. */
enum class RouteFailureReason {
  None,
  UnknownTarget,
  TargetUnavailable,
  NoCurrentObservation,
  TargetNodeMissing,
  NoGraphPath,
  BundleSequenceFailure,
};

/** Skeleton, bundles, and gate outcome for one requested target. */
struct RoutePreparationResult {
  bool success{false};
  RouteFailureReason failure_reason{RouteFailureReason::None};
  std::string message{};
  GraphPath skeleton_path{};
  BundleSequence bundle_sequence{};
  GatePreparationResult gate_preparation{};
};

}  // namespace mars::graph_bundle_management
