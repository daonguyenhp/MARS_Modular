# Module 2 Implementation Plan

## 1. Objective

Implement the ROS-independent core of Module 2 in C++17. The module consumes
the geometric perception produced by Module 1 and manages:

1. global open-point memory and paper-based ranking;
2. the accumulated visibility graph and graph search;
3. observation bundles, bundle history, ordered bundle sequences, and gate
   preparation for Module 3.

This plan covers implementation only. The detailed test plan, Gazebo setup,
RViz visualization, and full-system simulation will be agreed separately.

## 2. Chosen Technical Direction

Module 2 will follow the C++17 structure used by Module 1 on the `tuannghia`
branch.

Reasons:

- Module 1 already exposes C++ types and a C++ `PerceptionResult`.
- Direct C++ integration avoids maintaining an early C++ to Python binding.
- Module 1 already establishes a CMake library target and GTest-based project
  structure that Module 2 can follow.
- The core algorithm can remain independent of ROS, Gazebo, RViz, TF, and
  robot drivers.

All source code, public API names, documentation strings, diagnostics, and
comments added by Module 2 will be written in English.

## 2.1 Legacy-Code Reuse Policy

Module 2 algorithms will be ported from the inspected legacy implementation
only after their behavior has been checked against the paper and the WBS.
Algorithm behavior must not be invented when a verified legacy implementation
already exists.

Because the legacy implementation is mainly Python and Module 2 will be C++17,
reuse usually means a faithful logic port rather than literal text copying.
Adaptations are allowed only for types, ownership, error handling, deterministic
IDs, language differences, and the agreed module boundary.

Every ported implementation must follow this process:

1. identify the exact legacy file and function;
2. compare its behavior with the relevant paper section and WBS requirement;
3. separate the paper algorithm from robot-specific heuristics and ROS code;
4. port the smallest verified unit into Module 2;
5. document the source and any intentional adaptation in an English code
   comment near the implementation;
6. record any unresolved difference instead of choosing behavior silently.

A suitable provenance comment is:

```cpp
// Logic ported from <legacy file>::<function>.
// Verified against paper Section <section> and adapted to the Module 2 types.
```

The legacy source mapping is:

| Module 2 responsibility | Primary legacy source | Required verification |
|---|---|---|
| Active-point filtering | `online_open_sight_planner.py::_filter_active_open_points` | WBS MD2.1 and paper Sections 4.3/4.6 |
| Ranking | `online_open_sight_planner.py::_rank_points` and the older `online_sight_polygon.py::ranking_score` | Paper Section 4.3 formula; remove legacy bonuses and map-scale normalization |
| Open-point records and lifecycle | `online_memory_manager.py::make_open_point`, `merge_open_point_into_global`, `get_highest_ranked_active`, `classify_target`, and `invalidate_open_points_in_explored` | WBS MD2.1 and Algorithm 2 state transitions |
| Safe graph insertion | `dsfm_polygon_node.py::_graph_insert` | Paper Section 4.5; preserve the visibility/collision proof and remove ROS ownership |
| Skeleton search | `dsfm_polygon_node.py::_bfs_skeleton_path` | WBS MD2.2; legacy behavior is cost-based and must be named Dijkstra, while a real BFS is implemented separately |
| Bundle construction | `paper_bundle_tools.py::extract_paper_bundle` and `online_bundle_builder.py::build_bundle` | Paper Sections 3.1-3.2 and the visible-boundary output of Module 1 |
| Bundle storage and retrieval | `online_bundle_builder.py::BundleHistory` | WBS MD2.3; fix route-direction ambiguity during extraction |
| Escape geometry expectations | `online_escape_geometry.py` | Use only to understand the bundle-sequence contract with Module 3 |
| Gate and portal ordering ideas | `bundle_corridor.py` and `online_dsfm_successive_funnel_return_planner.py::prepare_return_gates_from_ordered_bundles` | Verify against the paper and the Module 3 contract before porting |
| Overall call order | Paper Algorithm 2 and the relevant calls in `dsfm_online_state_machine_node.py` | Use only the algorithm flow; do not port the ROS state machine into Module 2 |

The following legacy behavior must not be copied into the Module 2 core unless
the paper or an agreed requirement explicitly justifies it:

- ranking look-ahead, confidence, arc, or step bonuses;
- the legacy `distance_scale = 150` normalization;
- graph edges created only from distance without a visibility proof;
- artificial gates created perpendicular to an existing path;
- center-to-center link gates that are not supported by corridor geometry;
- ROS parameters, logging, mission state, map loading, and motor control;
- silent A* or straight-line fallback presented as a bundle/funnel result.

## 3. Scope

### 3.1 Included

- Ingest the current `PerceptionResult`, robot pose, goal, and observation ID.
- Store open points across observations.
- Merge duplicate open points within a configurable geometric tolerance.
- Track open-point lifecycle state.
- Remove inactive, visited, selected, or otherwise unavailable points from the
  active candidate set.
- Recompute the Section 4.3 ranking from the current robot position.
- Build the local visibility graph for every observation.
- Merge local graphs into one explored visibility graph.
- Search the graph for a skeleton route to a historical open point.
- Build one geometric bundle for every accepted observation.
- Store and retrieve bundles using observation and graph metadata.
- Extract bundles in the same direction as the selected skeleton route.
- Prepare validated gate data for Module 3.
- Expose small public interfaces that can be called without ROS.

### 3.2 Excluded

- Obstacle extraction, neighbor sights, closed sights, open sights, and open
  point generation. These belong to Module 1.
- Choosing the final robot state such as `EXPLORE`, `ESCAPE`, or `FAILED`.
- DAP, triangulation, funnel shortest path, and motion planning. These belong
  to Module 3.
- ROS 2 nodes, messages, TF conversion, Gazebo worlds, RViz markers, launch
  files, and hardware control. These belong to Module 4 or an adapter layer.
- Collision guarantees for a finite-size robot. Module 1 open points describe
  point geometry and do not by themselves guarantee a safe robot footprint.

## 4. Input and Output Boundary

### 4.1 Required Input

Each Module 2 update will receive:

```cpp
struct GraphBundleUpdate {
    std::uint64_t observation_id;
    mars::common::Pose2D current_pose;
    mars::common::Point2D goal;
    mars::perception_geometry::PerceptionResult perception;
};
```

The `perception.neighbor_sight.center` must agree with
`current_pose.position` within the configured coordinate tolerance.

Module 2 will use these Module 1 fields directly:

- `perception.open_points` for candidate exploration targets;
- `perception.open_sights` to preserve the source sight relationship;
- `perception.neighbor_sight.visible_boundaries` to build the current bundle;
- `perception.neighbor_sight.center` and `radius` to construct and validate
  local visibility relationships.

### 4.2 Public Output

Module 2 will expose:

- currently active ranked open points;
- the accumulated visibility graph;
- the skeleton path to a requested active historical point;
- the ordered bundle sequence associated with that path;
- prepared gates when a valid gate sequence can be derived;
- explicit failure information when a route, bundle sequence, or valid gate
  sequence cannot be constructed.

No function will silently manufacture a straight path or artificial gate when
the required geometric evidence is missing.

## 5. Proposed Directory Layout

```text
02_graph_bundle_management/
├── CMakeLists.txt
├── README.md
├── IMPLEMENTATION_PLAN.md
├── include/mars/graph_bundle_management/
│   ├── types.hpp
│   ├── ranking.hpp
│   ├── open_point_memory.hpp
│   ├── visibility_graph.hpp
│   ├── graph_search.hpp
│   ├── bundle_builder.hpp
│   ├── bundle_history.hpp
│   ├── sequence_extractor.hpp
│   ├── gate_preparation.hpp
│   └── graph_bundle_manager.hpp
├── internal/
│   ├── geometry_helpers.hpp
│   └── id_helpers.hpp
├── ranking_memory/
│   ├── ranking.cpp
│   └── open_point_memory.cpp
├── visibility_graph/
│   ├── visibility_graph.cpp
│   ├── graph_search.cpp
│   └── shared_sight_connector.cpp
├── bundle_sequences/
│   ├── bundle_builder.cpp
│   ├── bundle_history.cpp
│   ├── sequence_extractor.cpp
│   └── gate_preparation.cpp
└── pipeline/
    └── graph_bundle_manager.cpp
```

Tests will remain under the Module 2 test area, but their exact layout and
coverage are intentionally deferred from this plan.

## 6. Core Data Model

Define the Module 2 C++ types in
`include/mars/graph_bundle_management/types.hpp`. Reuse the canonical geometry
primitives from `mars/common/geometry_types.hpp` instead of defining another
`Point2D`, `Segment2D`, or `Pose2D`.

### 6.1 Open-Point Memory Types

```cpp
using OpenPointId = std::uint64_t;
using ObservationId = std::uint64_t;
using VisibilityNodeId = std::uint64_t;

enum class OpenPointStatus {
    Active,
    Selected,
    Reached,
    InactiveExplored,
    Invalid
};

struct OpenPointRecord {
    OpenPointId id;
    mars::common::Point2D point;
    double source_angle;
    std::optional<std::size_t> source_sight_index;
    ObservationId first_observation_id;
    ObservationId latest_observation_id;
    mars::common::Point2D source_center;
    OpenPointStatus status;
};

struct RankedOpenPoint {
    OpenPointId id;
    mars::common::Point2D point;
    double distance_to_goal;
    double angle_to_goal;
    double score;
};
```

The rank is an output calculated for the current pose. It will not be stored as
permanent memory because the angle term changes whenever the robot moves.

### 6.2 Visibility Graph Types

Every node will contain a kind and source metadata:

```cpp
enum class VisibilityNodeKind {
    ObservationCenter,
    OpenPoint,
    SharedSightAnchor
};
```

The graph stores node ID, position, kind, optional observation ID, and optional
open-point ID. Edges are undirected, contain Euclidean cost, and record why the
edge is considered visible. Stable IDs are used instead of comparing floating
point coordinates as identities.

### 6.3 Bundle Types

A bundle represents the paper's fan of line segments from one concurrent robot
position to ordered visible obstacle vertices.

```cpp
struct Bundle {
    ObservationId observation_id;
    VisibilityNodeId center_node_id;
    mars::common::Point2D concurrent_point;
    std::vector<mars::common::Point2D> ordered_vertices;
    std::vector<mars::common::Segment2D> segments;
    bool degenerate;
};
```

Bundle order must be deterministic. Vertices will be deduplicated with a
linear tolerance and sorted by normalized polar angle around the concurrent
point.

### 6.4 Sequence and Gate Types

`BundleSequence` stores the skeleton node path, the ordered observation IDs,
and bundles in traversal order. It must state whether the order is current to
target or target to current.

`Gate` stores left and right endpoints, source bundle metadata, sequence index,
orientation, and validation state. A gate with coincident endpoints is invalid.

## 7. Implementation Phases

### Phase 1: Establish the C++ Library Boundary

1. Add the Module 2 `CMakeLists.txt`.
2. Create the `mars_graph_bundle_management` library target.
3. Add the alias target `mars::graph_bundle_management`.
4. Require C++17 and enable the same compiler warnings as Module 1.
5. Add Module 1 and common C++ include paths through target dependencies.
6. Add the public headers and empty source units needed by later phases.
7. Update the Module 2 README with build instructions and ownership rules.

At the end of this phase, Module 2 must compile as an empty ROS-independent
library against the Module 1 C++ API.

### Phase 2: Finalize Shared Contracts

1. Add the Module 2 C++ data types described above.
2. Introduce `GraphBundleUpdate` so an update always includes perception,
   current pose, goal, and observation identity.
3. Keep Module 1 types unchanged unless a genuine missing field is identified.
4. Do not use the current Python `GraphBundleProvider.update(perception)` as the
   implementation contract because it lacks pose, goal, and observation data.
5. Document how Module 3 requests a skeleton route, bundle sequence, and gates.
6. Decide with the Module 3 owner whether gates are required as final portal
   pairs or whether an ordered bundle sequence is the authoritative handoff.

The last item is a contract decision. Gate geometry must not be invented before
Module 3 confirms the expected portal representation.

### Phase 3: Implement Section 4.3 Ranking and Global Memory

Create `ranking.hpp/.cpp` with small pure functions:

```cpp
double distance_to_goal(const Point2D& point, const Point2D& goal);
double angle_to_goal(const Point2D& point,
                     const Point2D& current,
                     const Point2D& goal);
double ranking_score(double distance, double angle,
                     double alpha, double beta);
std::vector<RankedOpenPoint> rank_open_points(...);
```

Use the paper formula:

```text
score(p, Ct) = alpha / distance(p, goal)
             + beta  / angle(p, Ct, goal)
```

Implementation rules:

- return positive infinity for a mathematically zero distance or angle;
- use configurable tolerances for numerical zero;
- reject negative or non-finite weights;
- compute the unsigned smallest angle in `[0, pi]`;
- recompute all active scores whenever the current pose or goal changes;
- apply a deterministic tie break using distance, discovery order, and stable
  open-point ID.

Create `OpenPointMemory` with the following responsibilities:

- ingest all open points from the current `PerceptionResult`;
- preserve their source observation, source center, source angle, and sight
  index;
- merge a new point with an existing record only when the geometric merge
  policy says they represent the same place;
- keep a stable ID after merging;
- mark records as selected, reached, explored, reactivated, or invalid through
  explicit methods;
- return active records without exposing mutable internal storage;
- mark points within a configurable visit radius as reached or explored;
- separate lifecycle state from ranking state.

The first implementation will contain only the paper/WBS behavior. Legacy
look-ahead bonuses, confidence bonuses, arc bonuses, and hard-coded map scales
will not be copied into the core ranking formula.

### Phase 4: Implement Section 4.5 Visibility Graph

Create one local graph for each observation:

- add an observation-center node for `C_t`;
- add or reuse one node for every active local open point;
- add a center-to-open-point edge supported by the current neighbor sight;
- attach the current observation ID to nodes and edges;
- calculate Euclidean edge cost;
- merge the local graph into the accumulated explored graph without duplicate
  nodes or duplicate undirected edges.

Create graph operations:

```cpp
VisibilityNodeId add_or_merge_node(...);
bool add_visible_edge(...);
std::optional<GraphPath> breadth_first_path(...);
std::optional<GraphPath> shortest_cost_path(...);
```

Both searches are useful and must be named accurately:

- breadth-first search satisfies the WBS requirement and minimizes edge count;
- Dijkstra search uses stored edge costs and minimizes geometric route length.

The caller must choose the policy explicitly. A Dijkstra implementation will
not be named BFS.

Add shared-sight connections as a separate component. It may add an anchor or
connection between observations only when overlap and visibility can be proven
from the two stored neighbor-sight snapshots. Distance alone is insufficient.
If the proof fails, the graph remains disconnected and reports no path.

### Phase 5: Implement Bundle Construction and History

Create `build_bundle(perception, observation_id, center_node_id)`.

The builder will:

1. read the occlusion-resolved `visible_boundaries` from Module 1;
2. collect both endpoints of every nondegenerate visible boundary;
3. remove duplicate vertices with the configured linear tolerance;
4. compute each vertex's normalized polar angle around the current center;
5. sort vertices deterministically;
6. create one segment from the center to each ordered vertex;
7. mark the bundle degenerate when it cannot represent useful sight geometry;
8. retain the observation and visibility-node association.

Create `BundleHistory` with lookup by observation ID and graph center-node ID.
It will reject duplicate observation IDs and will never erase all history as a
side effect of resetting ranking or open-point selection state.

### Phase 6: Extract an Ordered Bundle Sequence

Create `extract_bundle_sequence(graph_path, bundle_history, direction)`.

The extractor will:

1. accept a previously computed visibility-graph path;
2. walk the path in the requested travel direction;
3. resolve each path node to its associated observation;
4. ignore open-point and shared-anchor nodes that do not own a bundle;
5. remove only consecutive duplicate observation IDs;
6. retrieve the corresponding nondegenerate bundles;
7. preserve route order exactly;
8. return a structured failure when a required observation has no bundle.

This fixes the legacy ambiguity where a bundle range was always returned in
ascending pose-index order even when the robot was retreating in the opposite
direction.

### Phase 7: Prepare Gates for Module 3

Implement gate preparation only after the Module 2 to Module 3 contract is
confirmed in Phase 2.

The implementation must follow these rules:

- derive every gate from ordered bundle and visible-boundary geometry;
- maintain a consistent left/right orientation along the route;
- reject zero-width, non-finite, reversed, or unsupported gates;
- record which bundle or bundle pair produced each gate;
- preserve gate order;
- return a failure when a valid corridor cannot be constructed;
- never create gates merely by drawing perpendicular segments across a graph
  path or by connecting path centers.

Module 2 ends after producing validated ordered gate data. DAP and funnel path
generation remain in Module 3.

### Phase 8: Add the Orchestration Class

Create `GraphBundleManager` after the individual components are complete.

Its update flow will be:

```text
GraphBundleUpdate
  -> validate input consistency
  -> store observation snapshot
  -> ingest and update open-point memory
  -> create and merge the local visibility graph
  -> build and store the current bundle
  -> recompute active open-point ranking
  -> publish an immutable update summary
```

Route preparation will be a separate call:

```text
requested historical open point
  -> locate its graph node
  -> search the accumulated visibility graph
  -> extract bundles along the returned skeleton
  -> prepare gates if the gate contract is enabled
  -> return route preparation result
```

Keeping update and route preparation separate prevents every perception update
from running an unnecessary escape-path calculation.

### Phase 9: Documentation and Handoff

1. Document every public type and function in English.
2. Add one short example that constructs synthetic Module 1 input and calls
   Module 2 without ROS.
3. Document coordinate units, angle convention, tolerances, graph direction,
   and failure behavior.
4. Document the exact fields Module 3 may rely on.
5. Record unresolved integration decisions instead of hiding them in code.
6. Keep ROS and visualization instructions outside the core algorithm README.

## 8. Expected Public API Shape

The final API should remain small:

```cpp
class GraphBundleManager {
public:
    explicit GraphBundleManager(GraphBundleConfig config = {});

    UpdateSummary update(const GraphBundleUpdate& input);

    std::vector<RankedOpenPoint> ranked_open_points() const;
    const VisibilityGraph& visibility_graph() const;
    const OpenPointMemory& open_point_memory() const;
    const BundleHistory& bundle_history() const;

    RoutePreparationResult prepare_route(
        OpenPointId target,
        GraphSearchPolicy policy = GraphSearchPolicy::ShortestCost) const;
};
```

Lower-level functions and classes remain public where independent use is part
of the WBS acceptance criteria. The manager is a convenience composition, not
the only way to use ranking, graph, or bundle operations.

## 9. Configuration

Use one explicit configuration object rather than scattered constants:

```cpp
struct GraphBundleConfig {
    double rank_alpha;
    double rank_beta;
    double linear_epsilon;
    double angular_epsilon;
    double open_point_merge_radius;
    double visited_radius;
    GraphSearchPolicy search_policy;
};
```

Validate configuration at construction time. Do not copy the legacy
`distance_scale = 150` value because it belongs to a different coordinate
scale and changes the meaning of the paper formula.

## 10. Error Handling and Determinism

- Invalid configuration or malformed input throws `std::invalid_argument`.
- Normal algorithmic absence, such as no active point or no graph path, returns
  a typed result with a failure reason.
- Stored IDs and deterministic tie breaks make repeated identical input produce
  identical output.
- Geometry comparison uses documented tolerances; IDs never depend on exact
  floating point equality.
- No routine catches an error and silently substitutes an unverified route.

## 11. Integration Decisions Still Required

The following decisions must be made with the other module owners before final
integration, but they do not block ranking, memory, graph, or bundle-history
implementation:

1. Whether Module 1 will be merged before Module 2 development or linked with
   `add_subdirectory()` from its feature branch.
2. Whether the Python models remain documentation-only, are removed, or later
   receive a binding to the C++ core.
3. The exact gate/portal format Module 3 expects.
4. Whether Module 3 requests BFS or shortest-cost graph paths by default.
5. Which module owns finite robot-footprint collision validation.
6. How ROS messages and RViz markers mirror the C++ types in Module 4.

## 12. Completion Criteria

Implementation is complete when:

- Module 2 builds as a C++17 library without ROS;
- it consumes Module 1's C++ `PerceptionResult` directly;
- ranking and open-point memory are independently callable;
- the explored visibility graph supports explicit BFS and shortest-cost search;
- every stored observation can own a deterministic bundle;
- bundle sequences follow the actual skeleton-route direction;
- gate preparation either returns validated gates or an explicit failure;
- Module 3 can consume the documented outputs without reading Module 2's
  internal containers;
- all new code and comments are in English.

The verification criteria and simulation procedure will be added after the
team agrees on the testing strategy.
