#pragma once

#include <vector>

#include "mars/common/geometry_types.hpp"

namespace mars::demo_simulation {

struct BodyTwist {
  double linear_x{0.0};
  double angular_z{0.0};
};

/** Mode 1 sim_path_follower_node speeds and stop bands. */
struct FollowerConfig {
  double max_linear_speed{0.06};
  double max_angular_speed{0.35};
  double waypoint_tolerance{0.10};
  double goal_tolerance{0.08};
  double align_yaw{0.5};
};

class FakeOdom {
 public:
  explicit FakeOdom(mars::common::Pose2D pose = {});

  void integrate(BodyTwist command, double dt);
  const mars::common::Pose2D& pose() const noexcept { return pose_; }

 private:
  mars::common::Pose2D pose_{};
};

class PathFollower {
 public:
  explicit PathFollower(FollowerConfig config = {});

  void set_path(std::vector<mars::common::Point2D> path);
  bool path_done() const noexcept { return index_ >= path_.size(); }
  BodyTwist compute(const mars::common::Pose2D& pose);
  const FollowerConfig& config() const noexcept { return config_; }

 private:
  FollowerConfig config_{};
  std::vector<mars::common::Point2D> path_{};
  std::size_t index_{0};
};

}  // namespace mars::demo_simulation
