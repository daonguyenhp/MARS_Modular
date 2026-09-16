#pragma once

#include <vector>

namespace mars::common {

inline constexpr double pi = 3.141592653589793238462643383279502884;
inline constexpr double two_pi = 2.0 * pi;

// Cartesian meters in one caller-supplied frame. Angles are CCW radians from +X.
struct Point2D { double x{0.0}; double y{0.0}; };
struct Segment2D { Point2D start; Point2D end; };
// Two vertices: thin wall. Three or more: simple filled polygon, implicitly closed.
// Empty/one-vertex polygons and zero-length edges have no angular coverage.
struct Polygon2D { std::vector<Point2D> vertices; };
struct Circle2D { Point2D center; double radius{0.0}; };
struct Pose2D { Point2D position; double yaw{0.0}; };

// start in [0, 2*pi), sweep in [0, 2*pi]. Zero is empty; 2*pi is full.
// Wrapping requires no special sentinel: {350 degrees, 30 degrees} ends at 20.
struct AngularInterval { double start{0.0}; double sweep{0.0}; };

}  // namespace mars::common
