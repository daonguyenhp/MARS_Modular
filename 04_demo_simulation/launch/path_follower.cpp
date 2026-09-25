#include "path_follower.hpp"

#include <algorithm>
#include <cmath>

namespace mars::demo_simulation {
namespace {

double distance(mars::common::Point2D from, mars::common::Point2D to) {
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  return std::sqrt(dx * dx + dy * dy);
}

double wrap_angle(double angle) {
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) {
    angle -= 2.0 * pi;
  }
  while (angle < -pi) {
    angle += 2.0 * pi;
  }
  return angle;
}

}  // namespace

FakeOdom::FakeOdom(mars::common::Pose2D pose) : pose_(pose) {}

void FakeOdom::integrate(BodyTwist command, double dt) {
  pose_.position.x += command.linear_x * std::cos(pose_.yaw) * dt;
  pose_.position.y += command.linear_x * std::sin(pose_.yaw) * dt;
  pose_.yaw = wrap_angle(pose_.yaw + command.angular_z * dt);
}

PathFollower::PathFollower(FollowerConfig config) : config_(config) {}

void PathFollower::set_path(std::vector<mars::common::Point2D> path) {
  path_ = std::move(path);
  index_ = 0;
}

BodyTwist PathFollower::compute(const mars::common::Pose2D& pose) {
  while (index_ < path_.size()) {
    const bool last = index_ + 1 == path_.size();
    const double tolerance =
        last ? config_.goal_tolerance : config_.waypoint_tolerance;
    if (distance(pose.position, path_[index_]) >= tolerance) {
      break;
    }
    ++index_;
  }
  BodyTwist command;
  if (index_ >= path_.size()) {
    return command;
  }

  const mars::common::Point2D target = path_[index_];
  const double dx = target.x - pose.position.x;
  const double dy = target.y - pose.position.y;
  const double gap = std::sqrt(dx * dx + dy * dy);
  const double yaw_error = wrap_angle(std::atan2(dy, dx) - pose.yaw);
  command.angular_z = std::clamp(1.2 * yaw_error, -config_.max_angular_speed,
                                 config_.max_angular_speed);
  if (std::abs(yaw_error) < config_.align_yaw) {
    constexpr double pi = 3.14159265358979323846;
    const double speed_factor = 1.0 - std::abs(yaw_error) / pi;
    command.linear_x =
        std::min(gap, config_.max_linear_speed * speed_factor);
  }
  return command;
}

}  // namespace mars::demo_simulation
