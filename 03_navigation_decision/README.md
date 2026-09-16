# Module 3: Navigation Decision

- **WBS:** 3.0, MD3.1, MD3.2, MD3.3
- **Owner:** Member 3
- **Purpose:** Select exploration targets, manage blind-alley escape state, and prepare DAP/Funnel path planning.
- **Paper question:** When should the robot explore or escape, and is the return path no longer than the entry path?
- **Expected inputs:** Ranked open points, visibility graph, bundle sequence/gates, current pose, and goal.
- **Expected outputs:** Navigation state, selected target, and planned path.
- **Dependencies:** `common/types`; may consume Module 1 and Module 2 outputs; no ROS dependency in core logic.
- **Acceptance criteria:** Explicit deterministic state transitions and synthetic-gate testing support are prepared.
- **Current status:** Not started.

Subdirectories are reserved for explore, BAR escape, DAP/Funnel, and focused tests.