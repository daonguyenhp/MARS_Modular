# Module 2: Graph and Bundle Management

Module 2 is a ROS-independent C++17 library. It consumes the geometric output
of Module 1, keeps global exploration memory, builds an accumulated visibility
graph, and reconstructs ordered bundle sequences for Module 3.

## Build and verify

From the `MARS_Modular` directory:

```sh
cmake -S . -B build -DMARS_BUILD_TESTS=ON
cmake --build build
ctest --test-dir build --output-on-failure
```

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

The exact portal representation is still an integration decision. Therefore
`GraphBundleConfig::gate_contract_enabled` defaults to `false`, and gate
preparation reports `ContractNotEnabled`. Even if enabled, the current version
reports `UnsupportedGeometry`; it deliberately does not port the legacy
path-perpendicular or center-link gate heuristics.

## Ownership

This directory owns ranking, open-point lifecycle, explored visibility graph,
bundle construction/history, route-ordered bundle extraction, and gate
validation at the Module 2 boundary. Module 1 owns perception geometry;
Module 3 owns DAP, funnel, and motion planning; ROS, simulation, and
visualization remain outside this core.

See `examples/synthetic_update.cpp` for a complete synthetic call with no ROS
dependency.
