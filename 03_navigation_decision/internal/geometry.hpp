#pragma once

#include <cmath>
#include <stdexcept>
#include <string>

#include "types.hpp"

namespace mars::navigation_decision::internal {

inline bool finite_point(const Point2D& point) noexcept {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

inline void require_finite(const Point2D& point, const char* name) {
  if (!finite_point(point)) {
    throw std::invalid_argument(std::string(name) + " must be finite");
  }
}

inline double distance(const Point2D& first, const Point2D& second) noexcept {
  return std::hypot(first.x - second.x, first.y - second.y);
}

inline bool nearly_equal(const Point2D& first, const Point2D& second,
                         double epsilon) noexcept {
  return distance(first, second) <= epsilon;
}

/** Recast / Lee–Preparata signed area: (c-a) × (b-a). */
inline double tri_area2(const Point2D& a, const Point2D& b,
                        const Point2D& c) noexcept {
  return (c.x - a.x) * (b.y - a.y) - (b.x - a.x) * (c.y - a.y);
}

}  // namespace mars::navigation_decision::internal
