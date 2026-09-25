# Module Contracts

## Dependency Direction

The intended pipeline is:

`01_perception_geometry` -> `02_graph_bundle_management` -> `03_navigation_decision` -> `04_demo_simulation`

Core modules remain ROS-independent. ROS topics, messages, nodes, launch files, and visualization adapters belong in Module 4 or a future adapter layer.

## Data Flow

### Module 1: Perception Geometry

**Input**

- Current robot pose / center `Ct`
- Vision radius `r`
- Obstacle geometry

**Output**

- `NeighborSight`
- Closed sights
- Open sights
- Open points
- `PerceptionResult`

### Module 2: Graph and Bundle Management

**Input**

- `PerceptionResult`
- Current pose
- Goal
- Exploration history

**Output**

- Ranked open points
- `VisibilityGraph`
- `BundleSequence`
- Gates

### Module 3: Navigation Decision

**Input**

- Ranked open points
- `VisibilityGraph`
- Bundle sequence / gates
- Current pose
- Goal

**Output**

- `NavigationState`
- Selected target
- `PlannedPath`

### Module 4: Demo and Simulation

Consumes the outputs above and owns ROS 2 integration, simulation, visualization, launch files, and demos.

## Contract Rules

- Module 1 must not import Module 2 or Module 3.
- Module 2 may consume shared types and Module 1 outputs.
- Module 3 may consume Module 1 and Module 2 outputs.
- Module 4 may integrate all modules with ROS 2.
- Shared models in `common/types` must remain ROS-independent.
- Module 1 implements the pure C++17 perception pipeline. Module 2 implements
  ranking, the visibility graph, and bundle sequences. Module 3 implements
  Algorithm 2 (Explore / BAR / DAP-funnel). Module 4 loads the Mode 1 maps,
  follows each planned path with fake odometry, and publishes the marker
  layers for RViz from the ROS 1 package `ros1/mars_mode1_sim`.

## C++ Module 1 contract

`mars::perception_geometry::perceive(center, radius, obstacles)` returns a
`PerceptionResult`. Shared primitives live in
`common/cpp/include/mars/common/geometry_types.hpp`; perception outputs live in
`01_perception_geometry/include/mars/perception_geometry/types.hpp`.

Python contracts are preserved as scaffolding, not runtime bindings. C++
intervals use start/sweep to distinguish empty, wrapping, and full coverage.
Merged closed sights retain multiple visible boundary fragments. Open points
include their representative angle and optional source index. Radius and
obstacles are explicit inputs, rather than unspecified state in the Python
`PerceptionProvider.compute(center)` stub.

The synthetic API assumes complete obstacle geometry and a point observer in
free space. Future adapters must resolve frames, timestamps, and unknown sensor
coverage before supplying geometry. See Module 1's README for numerical and
invalid-input rules. ROS remains in a future integration layer.
