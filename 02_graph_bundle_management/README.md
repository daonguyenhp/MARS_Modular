# Module 2: Graph and Bundle Management

- **WBS:** 2.0, MD2.1, MD2.2, MD2.3
- **Owner:** Member 2
- **Purpose:** Manage exploration memory, visibility connectivity, and sequences of bundles/gates.
- **Paper question:** How does the robot remember explored space and reconstruct the bundle sequence needed to escape?
- **Expected inputs:** `PerceptionResult`, current pose, goal, and exploration history.
- **Expected outputs:** Ranked open points, visibility graph, bundle sequence, and gates.
- **Dependencies:** `common/types`; may consume Module 1 outputs; no ROS dependency in core logic.
- **Acceptance criteria:** Ranking, graph operations, and bundle preparation are independently callable and testable offline.
- **Current status:** Not started.

Subdirectories are reserved for ranking/memory, visibility graph, bundle sequences, and focused tests.