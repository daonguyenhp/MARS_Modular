# Module 1: Perception Geometry

Owner: Nghia. Tasks 1–6 implement a stateless C++17 geometry library. Inputs
are synthetic obstacle geometry, center, and radius. No ROS, TF, driver,
visualization, goal, ranking, memory, graph, bundle, planning, or motor control
is part of the core.

## Folder layout

Implementations and their tests are grouped by responsibility:

- `geometry/`: shared geometry operations and primitive tests.
- `neighbor_sight/`: disk clipping and nearest-boundary visibility.
- `closed_sights/`: blocked angular coverage and supporting boundaries.
- `open_sights/`: free angular sectors and their representative open points.
- `pipeline/`: complete perception pipeline and integration tests.
- `internal/`: private geometry and interval helpers shared by stages.
- `include/mars/perception_geometry/`: public API headers and module types;
  existing `<mars/perception_geometry/...>` includes remain valid.
- `tests/`: test-suite documentation linking to the tests in each feature folder.

The root `CMakeLists.txt` builds all stages into one library and all tests into
one test executable.

## Build and test

From `MARS_Modular/` (CMake >= 3.16, C++17 compiler):

```sh
cmake -S 01_perception_geometry -B .build/core -DCMAKE_BUILD_TYPE=Debug
cmake --build .build/core -j2
ctest --test-dir .build/core --output-on-failure
```

Tests use the host GTest CMake package (tested with GTest 1.11), with no download
at configure time. `-DBUILD_TESTING=OFF` builds the library without GTest.
Build output belongs in the Git-ignored `MARS_Modular/.build/` directory.
Other CMake projects can use `add_subdirectory()` and link the source-tree
target `mars::perception_geometry`; includes and C++17 propagate. Installed
package exports and ament packaging are deferred to integration.

## API and types

```cpp
#include <mars/perception_geometry/perception_pipeline.hpp>
using namespace mars::perception_geometry;
const Point2D center{0.0, 0.0};
const std::vector<Polygon2D> obstacles{
    {{{2.0, -1.0}, {2.0, 1.0}}}  // thin wall
};
const PerceptionResult result = perceive(center, 5.0, obstacles);
// result.neighbor_sight, closed_sights, open_sights, open_points
```

Individual stages are `compute_neighbor_sight(center, radius, obstacles)`,
`compute_closed_sights(neighbor)`, `compute_open_sights(closed)`, and
`compute_open_points(center, radius, open)`.

Canonical shared primitives live in
`../common/cpp/include/mars/common/geometry_types.hpp`: Point2D, Segment2D,
Polygon2D, Circle2D, Pose2D, AngularInterval. Module `types.hpp` aliases these
and defines NeighborSight, ClosedSight, OpenSight, OpenPoint, PerceptionResult.
Existing Python models remain unchanged; no language binding is provided.
C++ uses start/sweep intervals, a vector of supporting boundaries per merged
ClosedSight, and an explicit representative angle in OpenPoint.

## Input conventions

- One Cartesian frame, meters, +X/+Y, CCW radians, zero along +X. Transforms
  belong to the caller/future adapter.
- This synthetic API assumes **complete obstacle geometry** for the sensing
  region. No geometry means known free space. A sensor adapter must not turn
  unknown or unobserved space into free space.
- Two distinct polygon vertices represent a thin wall. Three or more represent
  a simple filled polygon, implicitly closed, with either winding accepted.
  Explicit closing vertices and consecutive duplicates are removed within
  linear tolerance. Empty/one-point inputs and zero-length edges are ignored.
- Self-intersecting, backtracking, and zero-area filled polygons are rejected.
  Distinct walls/polygons may intersect or overlap.
- The observer must be outside every filled polygon and farther than linear
  tolerance from every nondegenerate edge. Enclosing free space is represented
  by separate walls, not a filled polygon containing the observer.
- Radius must be finite and greater than `linear_epsilon`. Invalid values,
  topology, or observer placement throw `std::invalid_argument`. Unrepresentable
  arithmetic throws `std::overflow_error`; a lost limiting visibility hit throws
  `std::runtime_error` rather than silently becoming free space.

`AngularInterval{start, sweep}` uses start in [0, 2π) and sweep in [0, 2π]. Zero
is empty; 2π is full. `{350°, 30°}` wraps to 20°. Consumers reject noncanonical
intervals; callers can use `normalize_angle()`. Results are sorted by normalized
start; a wrapping interval may appear last. Full coverage is `{0, 2π}`.

## Research basis

Reference: *The Sequences of Bundles of Line Segments for Autonomous Robots
with Limited Vision Range to Escape from Blind Alley Regions*, Sections 4.1–4.2.
The inspected PDF is bundled in the external `dsfm_escape_navigation_paper_31_7`
package. Publication: https://doi.org/10.1016/j.robot.2025.105185.

The paper describes closed sights using visible obstacle boundaries and the
observer, open sights using sensing-circle arcs, and open points as arc
midpoints. Here, merged blocked sectors retain all supporting visible fragments;
they need not be single triangles. NeighborSight stores center, radius, and
nearest-boundary fragments as a compact radial representation rather than a
materialized free-space polygon. Goal selection and explored-sight memory are
outside this implementation.

## Neighbor Sight and occlusion

1. Validate geometry and translate to observer-relative coordinates.
2. Analytically clip edges to the sensing disk; ignore zero-length results.
3. Collect critical angles from clipped endpoints and pairwise edge crossings.
   Clipped endpoints include segment-circle intersections.
4. Partition the circle at those events. For each positive-width cell, cast
   one midpoint ray and select the nearest intersected segment.
5. Intersect that segment with the cell's limiting rays to recover its visible
   fragment. Join adjacent fragments of the same input edge, including at zero.

Between consecutive critical events the intersected edges and their depth
ordering cannot change. This is geometry-driven interval visibility, not a
fixed-resolution scan. Crossing events handle intersecting walls. ±epsilon
angle offsets are unnecessary because selection rays lie inside event cells.
Hidden geometry is removed before classification; partially occluded edges
can produce several fragments. Exact depth ties follow input order with the
same coincident geometry. Identical input gives deterministic output.
Worst-case time O(E³), space O(E²), for E clipped edges. Large-map acceleration
is deferred; this implementation favors inspectable correctness.

## Closed Sights, Open Sights, Open Points

Visible fragments project onto minor angular intervals. Wrapping intervals
are split for linear union, overlaps/touching intervals merge, and the circular
seam is rejoined. Supporting fragments remain attached. Direct callers must
supply occlusion-resolved NeighborSight boundaries: classification does not
prove visibility or repeat occlusion processing.

Open sights are the circular complement of blocked coverage. Empty closed
coverage gives one full open circle; total blockage gives no open sights.
Seam gaps are measured as one circular gap before applying tolerance.

For each open sight, `theta = normalize(start + sweep/2)` and
`point = center + radius * (cos(theta), sin(theta))`. OpenPoint contains this
point, angle, and optional source index. The vector API preserves order and
populates indices. Empty intervals have no midpoint and are rejected.
For the canonical full circle `{0, 2π}`, theta is π and the representative is
`(center.x-radius, center.y)`. This deterministic convention does not imply a
preferred movement direction. There is one point per sight, with no ranking,
selection, or goal-dependent subdivision.

## Numerical policy and limitations

- `linear_epsilon = 1e-9` meters: degenerate edges, endpoint comparisons,
  observer-boundary exclusion, and endpoint-hit tolerance.
- `angular_epsilon = 1e-10` radians: tiny event cells/sectors and adjacent
  coverage merging. Blocked sectors at/below this resolution are discarded;
  gaps at/below it are closed. This is not exact arithmetic at arbitrary scales.
- Intersection calculations use long-double intermediates. Ray parallelism
  uses 64 times double machine epsilon because directions use double trig;
  segment crossing predicates use long-double machine precision.
- Point obstacles, radial thin walls, and single-point disk tangencies have
  zero angular measure and produce no blocked sector. Limiting endpoints may
  be shared by adjacent sectors. Isolated blocked directions are not represented.
- Use local, meter-scale coordinates. Large absolute offsets may lose small
  features in double precision before translation into local coordinates.
- Open points are geometric representatives for a point observer, **not
  finite-footprint collision or motion-safety guarantees**. No inflation,
  uncertainty, dynamic prediction, or partial-sensor-coverage model is included.

## Verification and next integration work

Tests cover requested cases A–H, full enclosure, concave polygons, thin angular
obstacles, intersecting/overlapping walls, partial occlusion, tangency, wrapping,
invalid data, and transformation invariance. An independent ray-equation oracle
checks 12,000 directions on 30 seeded wall maps; sampled rays are used only in
verification, never as the production algorithm.

ROS 2 adapter, RViz2 markers, sensor/map conversion, timestamp/TF handling,
ROS 1 bridge, and physical-robot validation remain deferred. No external
Hiwonder runtime change is required for this pure core.
