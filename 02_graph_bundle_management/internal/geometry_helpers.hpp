#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "mars/common/geometry_types.hpp"

namespace mars::graph_bundle_management::internal {

inline bool finite(const mars::common::Point2D& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

inline double squared_distance(const mars::common::Point2D& first,
                               const mars::common::Point2D& second) noexcept {
  const double dx = first.x - second.x;
  const double dy = first.y - second.y;
  return dx * dx + dy * dy;
}

inline double distance(const mars::common::Point2D& first,
                       const mars::common::Point2D& second) noexcept {
  return std::hypot(first.x - second.x, first.y - second.y);
}

inline void require_finite(const mars::common::Point2D& point,
                           const char* name) {
  if (!finite(point)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

inline double cross(const mars::common::Point2D& first,
                    const mars::common::Point2D& second,
                    const mars::common::Point2D& third) noexcept {
  return (second.x - first.x) * (third.y - first.y) -
         (second.y - first.y) * (third.x - first.x);
}

inline int orientation(const mars::common::Point2D& first,
                       const mars::common::Point2D& second,
                       const mars::common::Point2D& third,
                       double epsilon) noexcept {
  const double value = cross(first, second, third);
  if (std::abs(value) <= epsilon) {
    return 0;
  }
  return value > 0.0 ? 1 : -1;
}

inline bool on_segment(const mars::common::Point2D& point,
                       const mars::common::Segment2D& segment,
                       double epsilon) noexcept {
  return std::abs(cross(segment.start, segment.end, point)) <= epsilon &&
         point.x >= std::min(segment.start.x, segment.end.x) - epsilon &&
         point.x <= std::max(segment.start.x, segment.end.x) + epsilon &&
         point.y >= std::min(segment.start.y, segment.end.y) - epsilon &&
         point.y <= std::max(segment.start.y, segment.end.y) + epsilon;
}

inline bool segments_intersect(const mars::common::Segment2D& first,
                               const mars::common::Segment2D& second,
                               double epsilon) noexcept {
  const int o1 = orientation(first.start, first.end, second.start, epsilon);
  const int o2 = orientation(first.start, first.end, second.end, epsilon);
  const int o3 = orientation(second.start, second.end, first.start, epsilon);
  const int o4 = orientation(second.start, second.end, first.end, epsilon);
  if (o1 != o2 && o3 != o4) {
    return true;
  }
  return (o1 == 0 && on_segment(second.start, first, epsilon)) ||
         (o2 == 0 && on_segment(second.end, first, epsilon)) ||
         (o3 == 0 && on_segment(first.start, second, epsilon)) ||
         (o4 == 0 && on_segment(first.end, second, epsilon));
}

inline bool point_near(const mars::common::Point2D& first,
                       const mars::common::Point2D& second,
                       double tolerance) noexcept {
  return squared_distance(first, second) <= tolerance * tolerance;
}

}  // namespace mars::graph_bundle_management::internal
