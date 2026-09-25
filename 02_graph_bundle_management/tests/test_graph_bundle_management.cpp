#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include "mars/graph_bundle_management/bundle_builder.hpp"
#include "mars/graph_bundle_management/gate_preparation.hpp"
#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/graph_bundle_management/graph_search.hpp"
#include "mars/graph_bundle_management/open_point_memory.hpp"
#include "mars/graph_bundle_management/ranking.hpp"
#include "mars/graph_bundle_management/sequence_extractor.hpp"
#include "mars/graph_bundle_management/shared_sight_connector.hpp"
#include "mars/perception_geometry/perception_pipeline.hpp"

namespace mgbm = mars::graph_bundle_management;
namespace mpg = mars::perception_geometry;
namespace mc = mars::common;

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

mpg::PerceptionResult perception(mc::Point2D center,
                                 std::vector<mpg::OpenPoint> open_points = {}) {
  mpg::PerceptionResult result;
  result.neighbor_sight.center = center;
  result.neighbor_sight.radius = 2.0;
  result.neighbor_sight.visible_boundaries = {
      {{center.x - 0.5, center.y + 1.0}, {center.x + 0.5, center.y + 1.0}}};
  result.open_points = std::move(open_points);
  return result;
}

void test_ranking() {
  const auto score = mgbm::ranking_score(2.0, 0.5, 4.0, 1.0);
  require(std::abs(score - 4.0) < 1.0e-12,
          "ranking formula does not match Section 4.3");
  require(std::isinf(mgbm::ranking_score(0.0, 1.0, 1.0, 1.0)),
          "zero distance must have positive infinite rank");
  require(std::isinf(mgbm::ranking_score(1.0, 0.0, 1.0, 1.0)),
          "zero angle must have positive infinite rank");
  bool rejected = false;
  try {
    static_cast<void>(mgbm::rank_open_points({}, {0.0, 0.0}, {1.0, 0.0},
                                              -1.0, 1.0));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "empty ranking input did not reject a negative weight");
}

void test_memory() {
  mgbm::OpenPointMemory memory(0.1);
  const auto first =
      perception({0.0, 0.0}, {{{1.0, 0.0}, 0.0, std::nullopt}});
  const auto second =
      perception({0.2, 0.0}, {{{1.05, 0.0}, 0.0, std::nullopt}});
  const auto first_ids = memory.ingest(first, 10);
  const auto second_ids = memory.ingest(second, 11);
  require(first_ids.front() == second_ids.front(),
          "geometric duplicate did not preserve its stable ID");
  require(memory.records().size() == 1,
          "geometric duplicate created a second record");
  require(memory.records().front().latest_observation_id == 11,
          "merged record did not retain latest observation metadata");
  require(memory.mark_selected(first_ids.front()), "selection failed");
  require(memory.active_records().empty(),
          "selected record remained in the active candidate set");
  require(memory.reactivate(first_ids.front()), "reactivation failed");
}

void test_search_policies() {
  mgbm::VisibilityGraph graph;
  const auto start = graph.add_or_merge_node(
      {0.0, 0.0}, mgbm::VisibilityNodeKind::ObservationCenter, 1);
  const auto long_hop = graph.add_or_merge_node(
      {5.0, 5.0}, mgbm::VisibilityNodeKind::ObservationCenter, 2);
  const auto short_one = graph.add_or_merge_node(
      {3.0, 0.0}, mgbm::VisibilityNodeKind::ObservationCenter, 3);
  const auto short_two = graph.add_or_merge_node(
      {6.0, 0.0}, mgbm::VisibilityNodeKind::ObservationCenter, 4);
  const auto goal = graph.add_or_merge_node(
      {10.0, 0.0}, mgbm::VisibilityNodeKind::ObservationCenter, 5);
  graph.add_visible_edge(start, long_hop, mgbm::VisibilityEvidence::SharedSightOverlap);
  graph.add_visible_edge(long_hop, goal, mgbm::VisibilityEvidence::SharedSightOverlap);
  graph.add_visible_edge(start, short_one, mgbm::VisibilityEvidence::SharedSightOverlap);
  graph.add_visible_edge(short_one, short_two, mgbm::VisibilityEvidence::SharedSightOverlap);
  graph.add_visible_edge(short_two, goal, mgbm::VisibilityEvidence::SharedSightOverlap);

  const auto bfs = mgbm::breadth_first_path(graph, start, goal);
  const auto dijkstra = mgbm::shortest_cost_path(graph, start, goal);
  require(bfs && bfs->node_ids.size() == 3,
          "BFS did not minimize edge count");
  require(dijkstra && dijkstra->node_ids.size() == 4,
          "Dijkstra did not minimize geometric cost");
  require(dijkstra->total_cost < bfs->total_cost,
          "Dijkstra route was not shorter than the BFS route");
}

void test_shared_sight_proof() {
  mpg::NeighborSight first;
  first.center = {0.0, 0.0};
  first.radius = 2.0;
  mpg::NeighborSight second;
  second.center = {1.0, 0.0};
  second.radius = 2.0;
  require(mgbm::neighbor_sights_prove_connection(first, second),
          "reciprocal clear neighbor sights did not connect");
  require(!mgbm::neighbor_sight_supports_point(first, {3.0, 0.0}),
          "an out-of-range point was treated as visible");
  first.visible_boundaries = {{{0.5, -0.5}, {0.5, 0.5}}};
  require(!mgbm::neighbor_sights_prove_connection(first, second),
          "a boundary-crossing connection was admitted");
}

void test_bundle_and_direction() {
  const auto input = perception({0.0, 0.0});
  const auto bundle = mgbm::build_bundle(input, 4, 8, 1.0e-9);
  require(!bundle.degenerate && bundle.segments.size() == 2,
          "visible boundary did not produce a useful bundle");
  require(bundle.ordered_vertices.front().x > bundle.ordered_vertices.back().x,
          "bundle vertices were not sorted by normalized polar angle");

  auto one_segment_input = perception({0.0, 0.0});
  one_segment_input.neighbor_sight.visible_boundaries = {
      {{0.0, 0.0}, {0.0, 1.0}}};
  const auto one_segment =
      mgbm::build_bundle(one_segment_input, 5, 9, 1.0e-9);
  require(!one_segment.degenerate && one_segment.segments.size() == 1,
          "a paper-valid one-segment bundle was marked degenerate");

  mgbm::VisibilityGraph graph;
  const auto first = graph.add_or_merge_node(
      {0.0, 0.0}, mgbm::VisibilityNodeKind::ObservationCenter, 1);
  const auto second = graph.add_or_merge_node(
      {1.0, 0.0}, mgbm::VisibilityNodeKind::ObservationCenter, 2);
  graph.add_visible_edge(first, second, mgbm::VisibilityEvidence::SharedSightOverlap);
  mgbm::BundleHistory history;
  history.add(mgbm::build_bundle(perception({0.0, 0.0}), 1, first));
  history.add(mgbm::build_bundle(perception({1.0, 0.0}), 2, second));
  const mgbm::GraphPath path{{first, second}, 1.0};
  const auto reverse = mgbm::extract_bundle_sequence(
      path, graph, history, mgbm::BundleSequenceDirection::TargetToCurrent);
  require(reverse.success && reverse.sequence.observation_ids ==
                                 std::vector<mgbm::ObservationId>({2, 1}),
          "bundle extraction did not preserve reverse route direction");
}

void test_manager_pipeline() {
  mgbm::GraphBundleManager manager;
  mgbm::GraphBundleUpdate first;
  first.observation_id = 1;
  first.current_pose = {{0.0, 0.0}, 0.0};
  first.goal = {3.0, 0.0};
  first.perception =
      perception({0.0, 0.0}, {{{2.0, 0.0}, 0.0, std::nullopt}});
  const auto first_summary = manager.update(first);
  require(first_summary.active_open_point_count == 1,
          "manager did not rank the active open point");

  mgbm::GraphBundleUpdate second;
  second.observation_id = 2;
  second.current_pose = {{0.25, 0.0}, 0.0};
  second.goal = {3.0, 0.0};
  second.perception = perception({0.25, 0.0});
  manager.update(second);
  const auto target = manager.ranked_open_points().front().id;
  const auto route = manager.prepare_route(target);
  require(route.success, "manager could not prepare a proven historical route");
  require(route.bundle_sequence.observation_ids ==
              std::vector<mgbm::ObservationId>({2, 1}),
          "manager bundle sequence does not follow the skeleton direction");
  require(!route.gate_preparation.success &&
              route.gate_preparation.failure_reason ==
                  mgbm::GateFailureReason::ContractNotEnabled,
          "unconfirmed gate contract did not produce an explicit failure");
}

void test_visible_boundary_gates() {
  mgbm::Bundle stuck;
  stuck.observation_id = 1;
  stuck.concurrent_point = {0.0, 0.0};
  stuck.ordered_vertices = {{0.0, 0.4}, {0.0, -0.4}};
  stuck.degenerate = false;
  mgbm::Bundle middle;
  middle.observation_id = 2;
  middle.concurrent_point = {1.0, 0.0};
  middle.ordered_vertices = {{1.0, 0.5}, {1.0, -0.5}};
  middle.degenerate = false;
  mgbm::Bundle goal;
  goal.observation_id = 3;
  goal.concurrent_point = {2.0, 0.0};
  goal.ordered_vertices = {{2.4, 0.4}, {2.4, -0.4}};
  goal.degenerate = false;
  const mars::common::Segment2D blocking{{2.2, -1.0}, {2.2, 1.0}};
  stuck.obstacle_edges = {blocking};
  middle.obstacle_edges = {blocking};
  goal.obstacle_edges = {blocking};
  mgbm::BundleSequence sequence;
  sequence.bundles = {stuck, middle, goal};

  const auto gates = mgbm::prepare_gates(sequence, true);
  require(gates.success && !gates.gates.empty(),
          "C* produced no sleeve edge");
  for (const auto& gate : gates.gates) {
    require(std::hypot(gate.left.x - gate.right.x, gate.left.y - gate.right.y) >
                1.0e-6,
            "common edge collapsed");
    const auto center_at = [](mars::common::Point2D point) {
      return std::hypot(point.x, point.y) <= 1.0e-6 ||
             std::hypot(point.x - 1.0, point.y) <= 1.0e-6 ||
             std::hypot(point.x - 2.0, point.y) <= 1.0e-6;
    };
    require(!(center_at(gate.left) && center_at(gate.right)),
            "center-to-center link was emitted as a gate");
    require(std::hypot(gate.left.x - 1.0, gate.left.y) > 1.0e-6 &&
                std::hypot(gate.right.x - 1.0, gate.right.y) > 1.0e-6,
            "portal touches the middle bundle center");
  }
}

void test_invalid_update_is_rejected_before_mutation() {
  mgbm::GraphBundleManager manager;
  mgbm::GraphBundleUpdate update;
  update.observation_id = 1;
  update.current_pose = {{0.0, 0.0}, 0.0};
  update.goal = {1.0, 0.0};
  update.perception =
      perception({0.0, 0.0}, {{{2.0, 0.0}, 0.0, std::nullopt}});
  update.perception.neighbor_sight.visible_boundaries.front().start.x =
      std::numeric_limits<double>::quiet_NaN();
  bool rejected = false;
  try {
    static_cast<void>(manager.update(update));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  require(rejected, "malformed update was not rejected");
  require(manager.open_point_memory().records().empty() &&
              manager.visibility_graph().nodes().empty() &&
              manager.bundle_history().size() == 0,
          "malformed update partially changed accumulated state");
}

void test_real_module1_to_module2_contract() {
  const mc::Point2D center{0.0, 0.0};
  const std::vector<mpg::Polygon2D> obstacles{
      {{{1.0, -0.5}, {1.0, 0.5}}}};
  const auto module1 = mpg::perceive(center, 2.0, obstacles);
  require(!module1.open_points.empty(),
          "Module 1 did not produce an open point for integration testing");

  mgbm::GraphBundleManager manager;
  mgbm::GraphBundleUpdate update;
  update.observation_id = 42;
  update.current_pose = {center, 0.0};
  update.goal = {-3.0, 0.0};
  update.perception = module1;
  const auto summary = manager.update(update);
  require(summary.active_open_point_count == module1.open_points.size(),
          "Module 2 did not ingest all Module 1 open points");
  const auto& record = manager.open_point_memory().records().front();
  require(std::abs(record.source_angle - module1.open_points.front().angle) <
              1.0e-12,
          "Module 2 did not preserve Module 1's source angle");
}

}  // namespace

int main() {
  try {
    test_ranking();
    test_memory();
    test_search_policies();
    test_shared_sight_proof();
    test_bundle_and_direction();
    test_manager_pipeline();
    test_visible_boundary_gates();
    test_invalid_update_is_rejected_before_mutation();
    test_real_module1_to_module2_contract();
  } catch (const std::exception& error) {
    std::cerr << "Test failure: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
  std::cout << "All graph-bundle management tests passed.\n";
  return EXIT_SUCCESS;
}
