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
- This initialization defines interfaces only; no paper algorithm is implemented yet.