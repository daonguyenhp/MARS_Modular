#include "laser_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace mars::online_jettank {
namespace {
using mars::common::Point2D;
using mars::common::Polygon2D;

double distance(Point2D a, Point2D b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double line_error(Point2D p, Point2D a, Point2D b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length = std::hypot(dx, dy);
  if (length < 1e-9) return distance(p, a);
  return std::abs(dx * (a.y - p.y) - (a.x - p.x) * dy) / length;
}

void simplify_cluster(const std::vector<Point2D>& points,
                      std::size_t first, std::size_t last,
                      std::vector<Polygon2D>& output) {
  if (last <= first) return;
  double error = 0.0;
  std::size_t split = first;
  for (std::size_t i = first + 1; i < last; ++i) {
    const double candidate = line_error(points[i], points[first], points[last]);
    if (candidate > error) {
      error = candidate;
      split = i;
    }
  }
  if (error > 0.035 && split > first && split < last) {
    simplify_cluster(points, first, split, output);
    simplify_cluster(points, split, last, output);
  } else if (distance(points[first], points[last]) > 0.025) {
    Polygon2D segment;
    segment.vertices = {points[first], points[last]};
    output.push_back(segment);
  }
}

}  // namespace

std::vector<Polygon2D> scan_to_obstacles(const LaserFrame& scan,
                                         mars::common::Pose2D pose,
                                         double vision_radius) {
  std::vector<Polygon2D> result;
  std::vector<Point2D> cluster;
  auto flush = [&] {
    if (cluster.size() >= 2) {
      simplify_cluster(cluster, 0, cluster.size() - 1, result);
    }
    cluster.clear();
  };

  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);
  Point2D previous{0.0, 0.0};
  bool have_previous = false;

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > vision_radius) {
      flush();
      have_previous = false;
      continue;
    }
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double local_x = r * std::cos(angle);
    const double local_y = r * std::sin(angle);
    const Point2D current{
        pose.position.x + local_x * cos_yaw - local_y * sin_yaw,
        pose.position.y + local_x * sin_yaw + local_y * cos_yaw,
    };
    if (have_previous && distance(previous, current) > 0.25) {
      flush();
    }
    cluster.push_back(current);
    previous = current;
    have_previous = true;
  }
  flush();
  return result;
}

bool target_visible_in_scan(const LaserFrame& scan,
                            mars::common::Pose2D pose,
                            mars::common::Point2D target,
                            double vision_radius) {
  const double target_dist = distance(pose.position, target);
  if (target_dist > vision_radius || target_dist < 1e-3) return false;
  double angle = std::atan2(target.y - pose.position.y, target.x - pose.position.x) - pose.yaw;
  while (angle <= -M_PI) angle += 2.0 * M_PI;
  while (angle > M_PI) angle -= 2.0 * M_PI;
  if (angle < scan.angle_min || angle > scan.angle_min + scan.ranges.size() * scan.angle_increment) {
    return false;
  }
  const int index = static_cast<int>(std::round((angle - scan.angle_min) / scan.angle_increment));
  if (index < 0 || static_cast<std::size_t>(index) >= scan.ranges.size()) return false;
  const double measured = scan.ranges[static_cast<std::size_t>(index)];
  return !std::isfinite(measured) || measured > target_dist - 0.12;
}

double forward_clearance(const LaserFrame& scan) {
  double clearance = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min) continue;
    double a = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    while (a <= -M_PI) a += 2.0 * M_PI;
    while (a > M_PI) a -= 2.0 * M_PI;
    if (std::abs(a) <= 0.35) {
      clearance = std::min(clearance, r * std::cos(a));
    }
  }
  return clearance;
}

}  // namespace mars::online_jettank
