#pragma once

#include <vector>

#include "mars/graph_bundle_management/types.hpp"

namespace mars::graph_bundle_management {

/** Return Euclidean distance from an open point to the goal. */
double distance_to_goal(const mars::common::Point2D& point,
                        const mars::common::Point2D& goal);

/** Return the unsigned smallest angle from current-to-point to current-to-goal. */
double angle_to_goal(const mars::common::Point2D& point,
                     const mars::common::Point2D& current,
                     const mars::common::Point2D& goal,
                     double linear_epsilon = 1.0e-9);

/** Evaluate the paper Section 4.3 inverse-distance/inverse-angle score. */
double ranking_score(double distance, double angle, double alpha, double beta,
                     double linear_epsilon = 1.0e-9,
                     double angular_epsilon = 1.0e-9);

/** Rank active records with deterministic tie breaking. */
std::vector<RankedOpenPoint> rank_open_points(
    const std::vector<OpenPointRecord>& records,
    const mars::common::Point2D& current,
    const mars::common::Point2D& goal,
    double alpha,
    double beta,
    double linear_epsilon = 1.0e-9,
    double angular_epsilon = 1.0e-9);

}  // namespace mars::graph_bundle_management
