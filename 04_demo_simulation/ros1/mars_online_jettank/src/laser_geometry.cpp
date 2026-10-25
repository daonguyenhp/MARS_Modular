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
  } else if (distance(points[first], points[last]) > 0.015) {
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

  // Allow points extending beyond vision_radius so continuous walls
  // are cleanly clipped by Module 1 clip_segment_to_circle at the circle boundary.
  const double max_detect_radius = std::min(scan.range_max, vision_radius + 0.60);

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(r) || r <= scan.range_min || r >= max_detect_radius) {
      flush();
      continue;
    }
    const double theta = pose.yaw + scan.angle_min +
                         static_cast<double>(i) * scan.angle_increment;
    const Point2D point{pose.position.x + r * std::cos(theta),
                        pose.position.y + r * std::sin(theta)};
    if (!cluster.empty() && distance(cluster.back(), point) > 0.22) flush();
    cluster.push_back(point);
  }
  flush();

  // Corner welding: snap adjacent/corner segment endpoints within 0.12m
  // to prevent artificial optical slits between perpendicular walls.
  for (std::size_t i = 0; i < result.size(); ++i) {
    if (result[i].vertices.size() < 2) continue;
    for (std::size_t j = i + 1; j < result.size(); ++j) {
      if (result[j].vertices.size() < 2) continue;
      for (int a = 0; a <= 1; ++a) {
        for (int b = 0; b <= 1; ++b) {
          Point2D& p_a = (a == 0) ? result[i].vertices.front() : result[i].vertices.back();
          Point2D& p_b = (b == 0) ? result[j].vertices.front() : result[j].vertices.back();
          if (distance(p_a, p_b) <= 0.12) {
            Point2D mid{(p_a.x + p_b.x) * 0.5, (p_a.y + p_b.y) * 0.5};
            p_a = mid;
            p_b = mid;
          }
        }
      }
    }
  }

  return result;
}

bool target_visible_in_scan(const LaserFrame& scan,
                            mars::common::Pose2D pose,
                            Point2D target, double vision_radius) {
  const double gap = distance(pose.position, target);
  if (gap > vision_radius || gap < 1e-6 || scan.ranges.empty() ||
      scan.angle_increment <= 0.0) return false;
  const double bearing = std::atan2(target.y - pose.position.y,
                                    target.x - pose.position.x) - pose.yaw;
  const double wrapped = std::atan2(std::sin(bearing), std::cos(bearing));
  const double index_float = (wrapped - scan.angle_min) / scan.angle_increment;
  const auto index = static_cast<long>(std::lround(index_float));
  if (index < 0 || index >= static_cast<long>(scan.ranges.size())) return false;
  const double seen = static_cast<double>(scan.ranges[static_cast<std::size_t>(index)]);
  return !std::isfinite(seen) || seen > gap + 0.12;
}

bool is_point_occluded_in_scan(const LaserFrame& scan,
                               mars::common::Pose2D pose,
                               Point2D target,
                               double margin) {
  const double gap = distance(pose.position, target);
  if (gap < 0.20 || scan.ranges.empty() || scan.angle_increment <= 0.0) {
    return false;
  }
  const double bearing = std::atan2(target.y - pose.position.y,
                                    target.x - pose.position.x) - pose.yaw;
  const double wrapped = std::atan2(std::sin(bearing), std::cos(bearing));
  const double index_float = (wrapped - scan.angle_min) / scan.angle_increment;
  const auto center_index = static_cast<long>(std::lround(index_float));

  int blocked_rays = 0;
  int valid_rays = 0;
  for (int offset = -4; offset <= 4; ++offset) {
    const long idx = center_index + offset;
    if (idx >= 0 && idx < static_cast<long>(scan.ranges.size())) {
      const double r = static_cast<double>(scan.ranges[static_cast<std::size_t>(idx)]);
      if (std::isfinite(r) && r > scan.range_min) {
        ++valid_rays;
        if (r < gap - margin) {
          ++blocked_rays;
        }
      }
    }
  }
  return (valid_rays > 0 && blocked_rays >= 2);
}

double forward_clearance(const LaserFrame& scan) {
  double closest = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    const double r = static_cast<double>(scan.ranges[i]);
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;
    const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    const double x = r * std::cos(angle);
    const double y = r * std::sin(angle);
    // Forward corridor for JetTank chassis (width 0.22m -> |y| <= 0.12m, front bumper at x = 0.13m)
    if (x > 0.10 && std::abs(y) <= 0.12) {
      const double gap_from_bumper = x - 0.13;
      if (gap_from_bumper < closest) {
        closest = gap_from_bumper;
      }
    }
  }
  return closest;
}

}  // namespace mars::online_jettank
