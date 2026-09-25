#include "types.hpp"

#include <cmath>
#include <stdexcept>

#include "internal/geometry.hpp"

namespace mars::navigation_decision {

double path_length(const std::vector<Point2D>& points) {
  if (points.size() < 2) {
    return 0.0;
  }
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    internal::require_finite(points[i - 1], "path point");
    internal::require_finite(points[i], "path point");
    length += internal::distance(points[i - 1], points[i]);
  }
  return length;
}

std::vector<Point2D> cap_last_hop(std::vector<Point2D> path, double max_step) {
  if (path.size() < 2 || max_step <= 0.0) {
    return path;
  }
  const Point2D& from = path[path.size() - 2];
  const Point2D& to = path.back();
  const double hop = internal::distance(from, to);
  if (hop > max_step) {
    const double t = max_step / hop;
    path.back() = {from.x + t * (to.x - from.x), from.y + t * (to.y - from.y)};
  }
  return path;
}

void validate_config(const NavigationConfig& config) {
  if (!std::isfinite(config.vision_radius) || config.vision_radius <= 0.0 ||
      !std::isfinite(config.goal_tolerance) || config.goal_tolerance <= 0.0 ||
      !std::isfinite(config.max_step) || config.max_step <= 0.0 ||
      !std::isfinite(config.arrival_tolerance) ||
      config.arrival_tolerance <= 0.0 ||
      !std::isfinite(config.linear_epsilon) || config.linear_epsilon < 0.0) {
    throw std::invalid_argument(
        "navigation tolerances must be finite and positive");
  }
  // Consecutive concurrent points must keep overlapping neighbor sights.
  if (config.max_step >= 2.0 * config.vision_radius) {
    throw std::invalid_argument(
        "max_step must be < 2 * vision_radius so consecutive scans overlap");
  }
}

}  // namespace mars::navigation_decision
