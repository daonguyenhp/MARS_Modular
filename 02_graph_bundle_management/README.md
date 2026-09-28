# Module 2: Graph and Bundle Management

Module 2 is a ROS-independent C++17 library. It consumes the geometric output
of Module 1, keeps global exploration memory, builds an accumulated visibility
graph, and reconstructs ordered bundle sequences for Module 3.

> **Hướng dẫn M2:** đọc [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)
> để xem luồng dữ liệu M1 → M2 → M3, cấu trúc implementation, cách chạy
> test/mô phỏng và bảng đối chiếu WBS MD2.1–MD2.3.

## Quick run

From `MARS_Modular/`, build and run the ROS-independent M2 test and example:

```sh
cmake -S . -B .build/m2 -DBUILD_TESTING=OFF -DMARS_BUILD_TESTS=ON -DMARS_DEMO_BUILD_TESTS=OFF
cmake --build .build/m2 --target mars_graph_bundle_management_tests mars_graph_bundle_management_example
.build/m2/02_graph_bundle_management/mars_graph_bundle_management_tests
.build/m2/02_graph_bundle_management/mars_graph_bundle_management_example
```

On a multi-config generator, add `--config Debug` to the build command and run
the executables under `02_graph_bundle_management/Debug/`. For the complete
Mode 1 simulation and ROS 1 RViz viewer, follow the guide linked above.
The guide also records the current gate-test failure observed on Windows/MinGW
with `main` at `76e8595`; do not treat a successful build as a passing test.

The library target is `mars_graph_bundle_management`; CMake consumers should
link the alias target `mars::graph_bundle_management`.

## Public boundary

The primary entry point is:

```cpp
#include "mars/graph_bundle_management/graph_bundle_manager.hpp"

mars::graph_bundle_management::GraphBundleManager manager;
auto summary = manager.update(update);
auto ranked = manager.ranked_open_points();
auto route = manager.prepare_route(ranked.front().id);
```

`GraphBundleUpdate` requires a stable observation ID, current pose, goal, and
one Module 1 `PerceptionResult`. Module 2 links the real
`mars::perception_geometry` target rather than carrying a duplicate boundary
type. The neighbor-sight center and current pose must agree within
`linear_epsilon`.

At this boundary Module 2 validates the sensing circle, visible boundaries,
open intervals, source-sight indices, and the radial geometry of each open
point. It preserves Module 1's normalized `OpenPoint::angle` as
`OpenPointRecord::source_angle`; it does not recalculate that value with a
separate convention.

Lower-level ranking, memory, graph search, bundle history, sequence extraction,
and gate-preparation APIs remain public for offline use and acceptance tests.

## Behavior and conventions

- Coordinates and distances are in metres.
- Pose yaw and sight angles are in radians.
- Bundle vertices are sorted by normalized polar angle in `[0, 2*pi)`.
- Open-point identity uses stable integer IDs; geometry is compared only with
  explicit tolerances.
- Ranking implements paper Section 4.3 exactly as
  `alpha / distance + beta / angle`. A zero denominator with a nonzero weight
  gives positive infinity. No legacy bonus or map-scale term is used.
- Breadth-first search minimizes edge count. Dijkstra search minimizes stored
  Euclidean edge cost. Callers choose the policy explicitly.
- Center-to-open-point edges are supported by the observation that generated
  the Module 1 open point. Cross-observation center edges require reciprocal
  neighbor-sight range and a boundary-intersection check; distance alone does
  not create an edge.
- Bundle sequences preserve the requested travel direction. Only consecutive
  duplicate observation IDs are removed.

## Lifecycle and failure behavior

Open points have explicit `Active`, `Selected`, `Reached`,
`InactiveExplored`, and `Invalid` states. Ranking is recomputed from active
records on every update and is never stored as permanent memory.

Malformed input and invalid configuration throw `std::invalid_argument`.
Normal absence, including an unknown target, disconnected graph, missing
bundle, or unsupported gate geometry, is returned as a typed failure.
No routine substitutes a straight-line route or artificial gate.

## Module 3 handoff

Module 3 may rely on these fields:

- `RoutePreparationResult::skeleton_path`: graph node IDs in travel order and
  total Euclidean edge cost;
- `RoutePreparationResult::bundle_sequence`: the same skeleton IDs, resolved
  observation IDs, and nondegenerate bundles in travel order;
- `RoutePreparationResult::gate_preparation`: validated gates or a typed,
  explicit failure.

When the contract is enabled, portals are the common edges of a
triangulation of C*. C* starts at bundle Cf from (11) and (12), keeps the
segments on the side of each turn that is at most pi, adds a segment at a
degenerate bundle, and trims segments of distinct bundles until they are
disjoint. The sleeve runs from the stuck end to the return end.
`GraphBundleConfig::gate_contract_enabled` defaults to `false`, and gate
preparation then reports `ContractNotEnabled`. Center-to-center links and
path-perpendicular segments are still not gates.

## Ownership

This directory owns ranking, open-point lifecycle, explored visibility graph,
bundle construction/history, route-ordered bundle extraction, and gate
validation at the Module 2 boundary. Module 1 owns perception geometry;
Module 3 owns DAP, funnel, and motion planning; ROS, simulation, and
visualization remain outside this core.

See `examples/synthetic_update.cpp` for a complete synthetic call with no ROS
dependency.
