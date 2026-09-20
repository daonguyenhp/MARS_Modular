#include "mars/graph_bundle_management/ranking.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "internal/geometry_helpers.hpp"

namespace mars::graph_bundle_management {

double distance_to_goal(const mars::common::Point2D& point,
                        const mars::common::Point2D& goal) {
  internal::require_finite(point, "point");
  internal::require_finite(goal, "goal");
  return internal::distance(point, goal);
}

double angle_to_goal(const mars::common::Point2D& point,
                     const mars::common::Point2D& current,
                     const mars::common::Point2D& goal,
                     double linear_epsilon) {
  internal::require_finite(point, "point");
  internal::require_finite(current, "current");
  internal::require_finite(goal, "goal");
  if (!std::isfinite(linear_epsilon) || linear_epsilon < 0.0) {
    throw std::invalid_argument("linear_epsilon must be finite and non-negative");
  }

  const double first_x = point.x - current.x;
  const double first_y = point.y - current.y;
  const double second_x = goal.x - current.x;
  const double second_y = goal.y - current.y;
  const double first_length = std::hypot(first_x, first_y);
  const double second_length = std::hypot(second_x, second_y);
  if (first_length <= linear_epsilon || second_length <= linear_epsilon) {
    return 0.0;
  }

  const double cosine = std::clamp(
      (first_x * second_x + first_y * second_y) /
          (first_length * second_length),
      -1.0, 1.0);
  return std::acos(cosine);
}

double ranking_score(double distance, double angle, double alpha, double beta,
                     double linear_epsilon, double angular_epsilon) {
  if (!std::isfinite(distance) || distance < 0.0 || !std::isfinite(angle) ||
      angle < 0.0 || !std::isfinite(alpha) || alpha < 0.0 ||
      !std::isfinite(beta) || beta < 0.0 || !std::isfinite(linear_epsilon) ||
      linear_epsilon < 0.0 || !std::isfinite(angular_epsilon) ||
      angular_epsilon < 0.0) {
    throw std::invalid_argument(
        "ranking inputs and tolerances must be finite and non-negative");
  }

  if (distance <= linear_epsilon || angle <= angular_epsilon) {
    return std::numeric_limits<double>::infinity();
  }
  const double distance_term = alpha == 0.0 ? 0.0 : alpha / distance;
  const double angle_term = beta == 0.0 ? 0.0 : beta / angle;
  return distance_term + angle_term;
}

std::vector<RankedOpenPoint> rank_open_points(
    const std::vector<OpenPointRecord>& records,
    const mars::common::Point2D& current,
    const mars::common::Point2D& goal,
    double alpha,
    double beta,
    double linear_epsilon,
    double angular_epsilon) {
  // Logic ported from online_open_sight_planner.py::_rank_points.
  // Verified against paper Section 4.3; legacy bonuses and map scaling are omitted.
  if (!std::isfinite(alpha) || alpha < 0.0 || !std::isfinite(beta) ||
      beta < 0.0 || !std::isfinite(linear_epsilon) ||
      linear_epsilon < 0.0 || !std::isfinite(angular_epsilon) ||
      angular_epsilon < 0.0) {
    throw std::invalid_argument(
        "ranking weights and tolerances must be finite and non-negative");
  }
  internal::require_finite(current, "current");
  internal::require_finite(goal, "goal");
  std::vector<RankedOpenPoint> ranked;
  ranked.reserve(records.size());
  for (const auto& record : records) {
    if (record.status != OpenPointStatus::Active) {
      continue;
    }
    const double distance = distance_to_goal(record.point, goal);
    const double angle = angle_to_goal(record.point, current, goal, linear_epsilon);
    ranked.push_back({record.id,
                      record.point,
                      distance,
                      angle,
                      ranking_score(distance, angle, alpha, beta,
                                    linear_epsilon, angular_epsilon),
                      record.first_observation_id});
  }

  std::sort(ranked.begin(), ranked.end(), [](const RankedOpenPoint& first,
                                             const RankedOpenPoint& second) {
    if (first.score != second.score) {
      return first.score > second.score;
    }
    if (first.distance_to_goal != second.distance_to_goal) {
      return first.distance_to_goal < second.distance_to_goal;
    }
    if (first.discovery_observation_id != second.discovery_observation_id) {
      return first.discovery_observation_id < second.discovery_observation_id;
    }
    return first.id < second.id;
  });
  return ranked;
}

}  // namespace mars::graph_bundle_management
