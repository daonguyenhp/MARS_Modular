# Perception Geometry Tests

GTest tests build with ordinary CMake and run through CTest without ROS.
See `../README.md` for commands.

- `test_geometry.cpp`: primitives, angles, intersections, degeneracy, invalid data.
- `test_neighbor_sight.cpp`: clipping, occlusion, crossing/overlapping walls,
  filled polygons, near-wall geometry, and invalid topology.
- `test_closed_sights.cpp`: coverage, merging, wrapping, boundary provenance.
- `test_open_sights.cpp`: complement, empty/full coverage, nested/touching
  intervals, tolerance, and seam gaps.
- `test_open_points.cpp`: midpoints, source indices, deterministic full circle.
- `test_pipeline.cpp`: cases A–H, concave U, enclosure, narrow obstacles,
  transformation invariance, repeatability, and 12,000 independent ray checks.
