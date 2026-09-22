#pragma once
#include <mars/perception_geometry/types.hpp>

namespace mars::perception_geometry {
// Absolute resolution for local, meter-scale geometry; not a collision margin.
inline constexpr double linear_epsilon = 1e-9;
inline constexpr double angular_epsilon = 1e-10;

// Invalid/nonfinite inputs throw std::invalid_argument. Arithmetic overflow
// throws std::overflow_error. Radius must exceed linear_epsilon.
void validate_point(const Point2D& point);
void validate_circle(const Circle2D& circle);
void validate_interval(const AngularInterval& interval);
double distance(const Point2D& a, const Point2D& b);
double normalize_angle(double angle);
// Empty intervals have no midpoint and are rejected.
double angular_midpoint(const AngularInterval& interval);
Point2D point_on_circle(const Circle2D& circle, double angle);

// Nearest forward hit, including endpoints/collinear overlap. A degenerate
// segment is a point for this helper (but ignored by the perception pipeline).
std::optional<Point2D> ray_segment_intersection(
    const Point2D& origin, double angle, const Segment2D& segment);
// Boundary intersections ordered from segment.start to segment.end.
std::vector<Point2D> segment_circle_intersections(
    const Segment2D& segment, const Circle2D& circle);
std::optional<Segment2D> clip_segment_to_circle(
    const Segment2D& segment, const Circle2D& circle);
}  // namespace mars::perception_geometry
