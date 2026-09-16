# Module 1: Perception Geometry

- **WBS:** 1.0, MD1.1, MD1.2, MD1.3
- **Owner:** Nghia
- **Purpose:** Convert local obstacle geometry into neighbor, closed, and open sights plus representative open points.
- **Paper question:** What can the robot see within vision radius `r`, where are obstacles, and which directions are open?
- **Expected inputs:** Robot center `Ct`, vision radius `r`, and obstacle/map geometry.
- **Expected outputs:** `NeighborSight`, closed sights, open sights, open points, and `PerceptionResult`.
- **Dependencies:** `common/types`; no ROS dependency in core logic.
- **Acceptance criteria:** Pure geometry logic is independently testable with synthetic maps/polygons.
- **Current status:** Not started.

Subdirectories are reserved for neighbor sight, closed sights, open sights, and focused tests.