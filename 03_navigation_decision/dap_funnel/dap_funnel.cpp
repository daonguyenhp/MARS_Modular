#include "dap_funnel.hpp"

#include "internal/geometry.hpp"

namespace mars::navigation_decision {
namespace {

struct Portal {
  Point2D left;
  Point2D right;
};

std::vector<Point2D> string_pull(const Point2D& start, const Point2D& goal,
                                 const std::vector<Portal>& portals,
                                 double epsilon) {
  std::vector<Portal> sequence;
  sequence.push_back({start, start});
  sequence.insert(sequence.end(), portals.begin(), portals.end());
  sequence.push_back({goal, goal});

  std::vector<Point2D> path{start};
  Point2D apex = start;
  Point2D left_pt = start;
  Point2D right_pt = start;
  std::size_t apex_i = 0;
  std::size_t left_i = 0;
  std::size_t right_i = 0;
  std::size_t i = 1;
  while (i < sequence.size()) {
    const Point2D left = sequence[i].left;
    const Point2D right = sequence[i].right;
    if (internal::tri_area2(apex, right_pt, right) <= 0.0) {
      if (internal::nearly_equal(apex, right_pt, epsilon) ||
          internal::tri_area2(apex, left_pt, right) > 0.0) {
        right_pt = right;
        right_i = i;
      } else {
        if (!internal::nearly_equal(path.back(), left_pt, epsilon)) {
          path.push_back(left_pt);
        }
        apex = left_pt;
        apex_i = left_i;
        left_pt = apex;
        right_pt = apex;
        left_i = apex_i;
        right_i = apex_i;
        i = apex_i + 1;
        continue;
      }
    }
    if (internal::tri_area2(apex, left_pt, left) >= 0.0) {
      if (internal::nearly_equal(apex, left_pt, epsilon) ||
          internal::tri_area2(apex, right_pt, left) < 0.0) {
        left_pt = left;
        left_i = i;
      } else {
        if (!internal::nearly_equal(path.back(), right_pt, epsilon)) {
          path.push_back(right_pt);
        }
        apex = right_pt;
        apex_i = right_i;
        left_pt = apex;
        right_pt = apex;
        left_i = apex_i;
        right_i = apex_i;
        i = apex_i + 1;
        continue;
      }
    }
    ++i;
  }
  if (!internal::nearly_equal(path.back(), goal, epsilon)) {
    path.push_back(goal);
  }
  return path;
}

}  // namespace

FunnelResult funnel_path(const Point2D& start, const Point2D& goal,
                         const std::vector<Gate>& gates,
                         const NavigationConfig& config) {
  validate_config(config);
  internal::require_finite(start, "start");
  internal::require_finite(goal, "goal");

  std::vector<Portal> portals;
  portals.reserve(gates.size());
  for (const auto& gate : gates) {
    internal::require_finite(gate.left, "gate left");
    internal::require_finite(gate.right, "gate right");
    if (internal::distance(gate.left, gate.right) <= config.linear_epsilon) {
      FunnelResult failed;
      failed.failure_reason = FunnelFailureReason::DegenerateGate;
      failed.message = "zero-width gate is not a portal";
      return failed;
    }
    portals.push_back({gate.left, gate.right});
  }

  FunnelResult result;
  result.path.points = string_pull(start, goal, portals, config.linear_epsilon);
  if (result.path.points.size() < 2) {
    result.failure_reason = FunnelFailureReason::EmptyPath;
    result.message = "funnel produced no path";
    return result;
  }
  result.success = true;
  result.path.length = path_length(result.path.points);
  return result;
}

bool return_not_longer(const std::vector<Point2D>& entry,
                       const std::vector<Point2D>& ret, double epsilon) {
  return path_length(ret) <= path_length(entry) + epsilon;
}

}  // namespace mars::navigation_decision
