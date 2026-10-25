#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "laser_geometry.hpp"
#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/perception_geometry/perception_pipeline.hpp"
#include "navigator.hpp"
#include "path_follower.hpp"

using namespace std::chrono_literals;
using mars::common::Point2D;
using mars::common::Pose2D;
using mars::graph_bundle_management::GraphBundleManager;
using mars::navigation_decision::NavigationDecision;
using mars::navigation_decision::NavigationObservation;
using mars::navigation_decision::Navigator;
using mars::online_gazebo::LaserFrame;
using visualization_msgs::msg::Marker;

namespace {
double distance(Point2D a, Point2D b) { return std::hypot(a.x - b.x, a.y - b.y); }

geometry_msgs::msg::Point point(Point2D p, double z = 0.05) {
  geometry_msgs::msg::Point result;
  result.x = p.x; result.y = p.y; result.z = z;
  return result;
}

Marker marker(const std::string& ns, int id, int type, const rclcpp::Time& stamp) {
  Marker out;
  out.header.frame_id = "odom";
  out.header.stamp = stamp;
  out.ns = ns; out.id = id; out.type = type; out.action = Marker::ADD;
  out.pose.orientation.w = 1.0;
  out.scale.x = 0.025; out.scale.y = 0.025; out.scale.z = 0.025;
  return out;
}

double wrap_angle(double angle) {
  constexpr double pi = 3.14159265358979323846;
  while (angle > pi) angle -= 2.0 * pi;
  while (angle < -pi) angle += 2.0 * pi;
  return angle;
}
}  // namespace

class OnlineNavigationNode : public rclcpp::Node {
 public:
  OnlineNavigationNode() : Node("mars_online_navigation") {
    const double world_goal_x = declare_parameter<double>("goal_x", 8.6);
    const double world_goal_y = declare_parameter<double>("goal_y", 6.8);
    // Gazebo's differential-drive odometry is local to the spawn pose. The
    // launch file therefore passes the world spawn offset so a world-frame
    // test goal can be converted into the node's odom frame.
    spawn_x_ = declare_parameter<double>("spawn_x", 0.0);
    spawn_y_ = declare_parameter<double>("spawn_y", 0.0);
    goal_ = {world_goal_x - spawn_x_, world_goal_y - spawn_y_};
    status_log_path_ = declare_parameter<std::string>(
        "status_log", "/tmp/mars_online_status.csv");
    config_.vision_radius = 1.6;
    config_.max_step = 0.70;
    config_.arrival_tolerance = 0.15;
    config_.goal_tolerance = 0.15;
    mars::demo_simulation::FollowerConfig follow_config;
    follow_config.max_linear_speed = 0.18;
    follow_config.max_angular_speed = 0.75;
    follow_config.waypoint_tolerance = 0.10;
    follow_config.goal_tolerance = 0.10;
    follower_ = std::make_unique<mars::demo_simulation::PathFollower>(follow_config);
    reset_algorithm();

    const auto sensor_qos = rclcpp::SensorDataQoS();
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", sensor_qos, [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          scan_.ranges = msg->ranges;
          scan_.angle_min = msg->angle_min;
          scan_.angle_increment = msg->angle_increment;
          scan_.range_min = msg->range_min;
          scan_.range_max = msg->range_max;
          scan_received_ = std::chrono::steady_clock::now();
          ++scan_sequence_;
        });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", sensor_qos, [this](nav_msgs::msg::Odometry::SharedPtr msg) {
          pose_.position = {msg->pose.pose.position.x, msg->pose.pose.position.y};
          const auto& q = msg->pose.pose.orientation;
          pose_.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
          pose_received_ = std::chrono::steady_clock::now();
          if (!have_pose_) {
            entry_pose_ = pose_.position;
            entry_path_ = {entry_pose_};
            entry_trace_.push_back(entry_pose_);
            have_pose_ = true;
          }
          if (trail_.empty() || distance(trail_.back(), pose_.position) > 0.025)
            trail_.push_back(pose_.position);
          auto& phase_trace = current_event_ == "FUNNEL" ? return_trace_ : entry_trace_;
          if (active_ && (phase_trace.empty() ||
                          distance(phase_trace.back(), pose_.position) > 0.025))
            phase_trace.push_back(pose_.position);
          geometry_msgs::msg::TransformStamped tf;
          tf.header.stamp = msg->header.stamp;
          tf.header.frame_id = "odom";
          tf.child_frame_id = "base_link";
          tf.transform.translation.x = msg->pose.pose.position.x;
          tf.transform.translation.y = msg->pose.pose.position.y;
          tf.transform.rotation = q;
          tf_broadcaster_->sendTransform(tf);
        });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          if (msg->header.frame_id != "odom" && msg->header.frame_id != "map") {
            RCLCPP_WARN(get_logger(), "Goal must use odom or map frame"); return;
          }
          goal_ = {msg->pose.position.x, msg->pose.position.y};
          active_ = false; terminal_ = false;
          if (have_pose_) { entry_pose_ = pose_.position; entry_path_ = {entry_pose_}; }
          reset_algorithm();
          RCLCPP_INFO(get_logger(), "New goal %.2f %.2f", goal_.x, goal_.y);
        });
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/mars_online/status", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/mars_online/markers", 10);
    trail_pub_ = create_publisher<nav_msgs::msg::Path>("/mars_online/trail", 10);
    entry_pub_ = create_publisher<nav_msgs::msg::Path>("/mars_online/entry_path", 10);
    return_pub_ = create_publisher<nav_msgs::msg::Path>("/mars_online/return_path", 10);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("/mars_online/planned_path", 10);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/mars_online/observed_map", 10);
    observed_map_.header.frame_id = "odom";
    observed_map_.info.resolution = 0.05;
    observed_map_.info.width = 200;
    observed_map_.info.height = 200;
    observed_map_.info.origin.orientation.w = 1.0;
    observed_map_.data.assign(200 * 200, -1);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = create_wall_timer(50ms, [this] { tick(); });
    open_status_log();
    RCLCPP_INFO(get_logger(), "Online MARS: waiting for real Gazebo /scan and /odom");
  }

 private:
  void reset_algorithm() {
    mars::graph_bundle_management::GraphBundleConfig bundle_config;
    bundle_config.gate_contract_enabled = true;
    manager_ = std::make_unique<GraphBundleManager>(bundle_config);
    navigator_ = std::make_unique<Navigator>(config_);
    observation_id_ = 0;
    scan_used_ = 0;
    recovery_phase_ = RecoveryPhase::None;
    recovery_attempts_ = 0;
    blocked_targets_.clear();
    current_event_ = "WAIT_SENSORS";
  }

  bool sensors_fresh() const {
    if (!have_pose_ || scan_.ranges.empty()) return false;
    const auto now = std::chrono::steady_clock::now();
    return now - scan_received_ < 1500ms && now - pose_received_ < 1500ms;
  }

  void stop() { cmd_pub_->publish(geometry_msgs::msg::Twist{}); }

  enum class RecoveryPhase { None, Backoff, Turn };

  void begin_recovery() {
    remember_blocked_target();
    ++recovery_attempts_;
    if (recovery_attempts_ > 5) {
      stop();
      terminal_ = true;
      active_ = false;
      current_event_ = "SAFETY_BLOCKED";
      RCLCPP_ERROR(get_logger(),
                   "Repeated obstacle recovery failed; stopping safely");
      return;
    }

    const double left = mars::online_gazebo::sector_clearance(scan_, 0.35, 1.45);
    const double right = mars::online_gazebo::sector_clearance(scan_, -1.45, -0.35);
    // Turn toward the side with more measured free space. If both sectors are
    // unknown, prefer left so the recovery is deterministic.
    turn_sign_ = left >= right ? 1.0 : -1.0;
    recovery_phase_ = RecoveryPhase::Backoff;
    recovery_deadline_ = std::chrono::steady_clock::now() + 700ms;
    active_ = false;
    current_event_ = "RECOVERY_BACKOFF";
    stop();
    RCLCPP_WARN(get_logger(),
                "Obstacle at %.2f m; backing up and turning %s (left %.2f, right %.2f)",
                mars::online_gazebo::forward_clearance(scan_),
                turn_sign_ > 0.0 ? "left" : "right", left, right);
  }

  struct BlockedTarget {
    Point2D point{};
    std::uint64_t expires_scan{0};
  };

  void remember_blocked_target() {
    if (planned_.size() < 2) return;
    const Point2D target = planned_[1];
    for (const auto& blocked : blocked_targets_) {
      if (distance(blocked.point, target) < 0.30) return;
    }
    blocked_targets_.push_back({target, scan_sequence_ + 40});
    RCLCPP_WARN(get_logger(),
                "Marking waypoint (%.2f, %.2f) blocked until scan %zu",
                target.x, target.y,
                static_cast<std::size_t>(scan_sequence_ + 40));
  }

  bool is_blocked_target(Point2D point) const {
    for (const auto& blocked : blocked_targets_) {
      if (scan_sequence_ <= blocked.expires_scan &&
          distance(blocked.point, point) < 0.45) {
        return true;
      }
    }
    return false;
  }

  void run_recovery() {
    const auto now = std::chrono::steady_clock::now();
    geometry_msgs::msg::Twist command;
    if (recovery_phase_ == RecoveryPhase::Backoff) {
      command.linear.x = -0.08;
      cmd_pub_->publish(command);
      if (now >= recovery_deadline_) {
        recovery_phase_ = RecoveryPhase::Turn;
        recovery_deadline_ = now + 2500ms;
        recovery_target_yaw_ = wrap_angle(pose_.yaw + turn_sign_ * 0.95);
        current_event_ = "RECOVERY_TURN";
      }
      return;
    }

    if (recovery_phase_ == RecoveryPhase::Turn) {
      const double error = wrap_angle(recovery_target_yaw_ - pose_.yaw);
      command.angular.z = turn_sign_ * 0.55;
      cmd_pub_->publish(command);
      if (std::abs(error) < 0.12 || now >= recovery_deadline_) {
        stop();
        recovery_phase_ = RecoveryPhase::None;
        scan_used_ = scan_sequence_;  // wait for a fresh view after turning
        current_event_ = "REPLAN";
        RCLCPP_INFO(get_logger(), "Recovery complete; replanning from fresh scan");
      }
    }
  }

  const char* recovery_phase_name() const {
    switch (recovery_phase_) {
      case RecoveryPhase::Backoff: return "BACKOFF";
      case RecoveryPhase::Turn: return "TURN";
      case RecoveryPhase::None: return "NONE";
    }
    return "UNKNOWN";
  }

  void open_status_log() {
    if (status_log_path_.empty()) return;
    try {
      const std::filesystem::path path(status_log_path_);
      if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
      status_log_.open(path, std::ios::out | std::ios::trunc);
      if (!status_log_) {
        RCLCPP_WARN(get_logger(), "Cannot open status log: %s", status_log_path_.c_str());
        return;
      }
      status_log_ << "wall_ms,ros_ns,event,x,y,yaw,scan_sequence,observation_id,"
                     "active,recovery_phase,front_clearance,left_clearance,"
                     "right_clearance,open_points,planned_points,trail_points\n";
      status_log_.flush();
      RCLCPP_INFO(get_logger(), "Writing status log to %s", status_log_path_.c_str());
    } catch (const std::exception& error) {
      RCLCPP_WARN(get_logger(), "Cannot initialize status log: %s", error.what());
    }
  }

  void log_status_if_changed(const rclcpp::Time& stamp) {
    if (!status_log_.is_open()) return;
    if (scan_sequence_ == logged_scan_sequence_ && current_event_ == logged_event_)
      return;
    const auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    const double front = mars::online_gazebo::forward_clearance(scan_);
    const double left = mars::online_gazebo::sector_clearance(scan_, 0.35, 1.45);
    const double right = mars::online_gazebo::sector_clearance(scan_, -1.45, -0.35);
    status_log_ << wall_ms << ',' << stamp.nanoseconds() << ',' << current_event_ << ','
                << pose_.position.x << ',' << pose_.position.y << ',' << pose_.yaw << ','
                << scan_sequence_ << ',' << observation_id_ << ',' << (active_ ? 1 : 0)
                << ',' << recovery_phase_name() << ',' << front << ',' << left << ','
                << right << ',' << last_open_points_ << ',' << planned_.size() << ','
                << trail_.size() << '\n';
    status_log_.flush();
    logged_scan_sequence_ = scan_sequence_;
    logged_event_ = current_event_;
  }

  void tick() {
    if (!sensors_fresh()) {
      stop();
      current_event_ = "WAIT_SENSORS";
      publish_visuals();
      return;
    }
    if (terminal_) { stop(); publish_visuals(); return; }
    if (scan_sequence_ > map_scan_used_) {
      update_observed_map();
      map_scan_used_ = scan_sequence_;
    }
    if (recovery_phase_ != RecoveryPhase::None) {
      run_recovery();
      publish_visuals();
      return;
    }
    if (active_) {
      const auto command = follower_->compute(pose_);
      if (follower_->path_done()) {
        stop();
        active_ = false;
        if (current_event_ == "EXPLORE" || current_event_ == "GOAL_VISIBLE")
          entry_path_.push_back(pose_.position);
      } else {
        if (distance(pose_.position, progress_pose_.position) > 0.03 ||
            std::abs(std::atan2(std::sin(pose_.yaw - progress_pose_.yaw),
                                std::cos(pose_.yaw - progress_pose_.yaw))) > 0.15) {
          progress_pose_ = pose_;
          progress_time_ = std::chrono::steady_clock::now();
        }
        if (std::chrono::steady_clock::now() - progress_time_ > 15s) {
          stop();
          active_ = false;
          terminal_ = true;
          current_event_ = "MOTION_STALLED";
          RCLCPP_ERROR(get_logger(), "Robot cannot follow selected path");
          publish_visuals();
          return;
        }
        geometry_msgs::msg::Twist twist;
        // Robot radius 0.14 m. Front obstacle below 0.23 m => stop safely.
        if (command.linear_x > 0 && mars::online_gazebo::forward_clearance(scan_) < 0.23) {
          begin_recovery();
        } else {
          if (command.linear_x > 0) {
            recovery_attempts_ = 0;
          }
          twist.linear.x = command.linear_x;
          twist.angular.z = command.angular_z;
          cmd_pub_->publish(twist);
        }
      }
    } else if (scan_sequence_ > scan_used_) {
      try {
        plan_from_current_scan();
      } catch (const std::exception& error) {
        stop();
        terminal_ = true;
        current_event_ = "PLANNER_ERROR";
        RCLCPP_ERROR(get_logger(), "Online perception/planning failed: %s", error.what());
      }
      scan_used_ = scan_sequence_;
    }
    publish_visuals();
  }

  void update_observed_map() {
    // RViz map is accumulated from actual LaserScan only; the planner does not
    // read this grid or the Gazebo wall file.
    auto index = [&](double x, double y) -> int {
      const int ix = static_cast<int>(std::floor(x / 0.05));
      const int iy = static_cast<int>(std::floor(y / 0.05));
      return ix >= 0 && ix < 200 && iy >= 0 && iy < 200 ? iy * 200 + ix : -1;
    };
    for (std::size_t i = 0; i < scan_.ranges.size(); i += 2) {
      const double measured = scan_.ranges[i];
      const bool hit = std::isfinite(measured) && measured < scan_.range_max - 0.02;
      const double extent = hit ? measured : scan_.range_max;
      const double theta = pose_.yaw + scan_.angle_min + i * scan_.angle_increment;
      for (double r = 0.0; r < extent - 0.04; r += 0.05) {
        const int cell = index(pose_.position.x + r * std::cos(theta),
                               pose_.position.y + r * std::sin(theta));
        if (cell >= 0 && observed_map_.data[static_cast<std::size_t>(cell)] != 100)
          observed_map_.data[static_cast<std::size_t>(cell)] = 0;
      }
      if (hit) {
        const int cell = index(pose_.position.x + measured * std::cos(theta),
                               pose_.position.y + measured * std::sin(theta));
        if (cell >= 0) observed_map_.data[static_cast<std::size_t>(cell)] = 100;
      }
    }
  }

  void plan_from_current_scan() {
    const auto obstacles = mars::online_gazebo::scan_to_obstacles(
        scan_, pose_, config_.vision_radius);
    const auto perception = mars::perception_geometry::perceive(
        pose_.position, config_.vision_radius, obstacles);
    mars::graph_bundle_management::GraphBundleUpdate update;
    update.observation_id = ++observation_id_;
    update.current_pose = pose_;
    update.goal = goal_;
    update.perception = perception;
    manager_->update(update);

    NavigationObservation observation;
    observation.pose = pose_.position;
    observation.goal = goal_;
    observation.goal_visible = mars::online_gazebo::target_visible_in_scan(
        scan_, pose_, goal_, config_.vision_radius);
    observation.entry_pose = entry_pose_;
    observation.entry_path = entry_path_;
    observation.ranked = manager_->ranked_open_points();
    observation.ranked.erase(
        std::remove_if(observation.ranked.begin(), observation.ranked.end(),
                       [this](const auto& candidate) {
                         return is_blocked_target(candidate.point);
                       }),
        observation.ranked.end());
    if (navigator_->return_target())
      observation.at_return_target =
          distance(pose_.position, *navigator_->return_target()) <= config_.arrival_tolerance;
    for (const auto& ranked : observation.ranked) {
      mars::navigation_decision::EscapeCandidate candidate;
      candidate.id = ranked.id;
      candidate.point = ranked.point;
      candidate.score = ranked.score;
      candidate.owner_pose = pose_.position;
      if (const auto* record = manager_->open_point_memory().find(ranked.id))
        candidate.owner_pose = record->source_center;
      observation.escape_candidates.push_back(candidate);
      const auto prepared = manager_->prepare_route(ranked.id);
      if (!prepared.success) continue;
      mars::navigation_decision::ReachableRoute route;
      route.id = ranked.id;
      for (const auto id : prepared.skeleton_path.node_ids)
        if (const auto* node = manager_->visibility_graph().find_node(id))
          route.skeleton.push_back(node->position);
      if (route.skeleton.size() >= 2) observation.routes.push_back(std::move(route));
    }
    // Match the existing Mode 1 mission: do not treat revisiting a driven
    // sensing point as new exploration of the alley.
    observation.routes.erase(std::remove_if(observation.routes.begin(), observation.routes.end(),
      [&](const mars::navigation_decision::ReachableRoute& route) {
        const auto steps = mars::navigation_decision::cap_last_hop(route.skeleton, config_.max_step);
        for (const auto& visited : entry_path_) {
          if (distance(visited, pose_.position) < 1e-9) continue;
          for (std::size_t i = 1; i < steps.size(); ++i)
            if (distance(visited, steps[i]) <= config_.arrival_tolerance) return true;
        }
        return false;
      }), observation.routes.end());
    if (observation.routes.empty()) {
      observation.ranked.clear();
      observation.escape_candidates.clear();
    }
    if (const auto gates = manager_->return_gates(); gates.success)
      observation.gates = gates.gates;

    auto decision = navigator_->decide(observation);
    if (decision.event == "BAR") decision = navigator_->decide(observation);
    current_event_ = decision.event;
    last_open_points_ = perception.open_points.size();
    planned_ = decision.planned_path.points;
    RCLCPP_INFO(get_logger(), "scan %zu, walls %zu, open %zu, event %s, path %zu",
                static_cast<std::size_t>(observation_id_), obstacles.size(),
                perception.open_points.size(), current_event_.c_str(), planned_.size());
    if (decision.state == mars::navigation_decision::NavigationState::GoalReached ||
        decision.state == mars::navigation_decision::NavigationState::Failed ||
        decision.event == "FUNNEL_FAIL") {
      terminal_ = true;
      stop();
      return;
    }
    if (decision.event == "RETURN_REACHED") return;
    if (planned_.size() < 2) {
      current_event_ = "NO_VALID_PATH";
      terminal_ = true;
      stop();
      return;
    }
    follower_->set_path(planned_);
    progress_pose_ = pose_;
    progress_time_ = std::chrono::steady_clock::now();
    active_ = true;
  }

  void publish_visuals() {
    const auto stamp = now();
    std_msgs::msg::String status;
    status.data = current_event_;
    status_pub_->publish(status);
    log_status_if_changed(stamp);
    if (!have_pose_) return;
    auto path_message = [&](const std::vector<Point2D>& points) {
      nav_msgs::msg::Path path;
      path.header.frame_id = "odom";
      path.header.stamp = stamp;
      for (const auto p : points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
      return path;
    };
    trail_pub_->publish(path_message(trail_));
    entry_pub_->publish(path_message(entry_trace_));
    return_pub_->publish(path_message(return_trace_));
    plan_pub_->publish(path_message(planned_));
    observed_map_.header.stamp = stamp;
    map_pub_->publish(observed_map_);

    visualization_msgs::msg::MarkerArray markers;
    auto clear = marker("clear", 0, Marker::LINE_LIST, stamp);
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);
    auto robot = marker("robot", 0, Marker::CYLINDER, stamp);
    robot.pose.position = point(pose_.position, 0.16);
    robot.scale.x = 0.28; robot.scale.y = 0.28; robot.scale.z = 0.20;
    robot.color.r = 0.05; robot.color.g = 0.30; robot.color.b = 0.85; robot.color.a = 0.8;
    markers.markers.push_back(robot);
    auto goal = marker("goal", 0, Marker::SPHERE, stamp);
    goal.pose.position = point(goal_, 0.12);
    goal.scale.x = 0.15; goal.scale.y = 0.15; goal.scale.z = 0.15;
    goal.color.r = 0.95; goal.color.g = 0.15; goal.color.b = 0.1; goal.color.a = 1;
    markers.markers.push_back(goal);
    auto range = marker("vision", 0, Marker::LINE_STRIP, stamp);
    range.scale.x = 0.012;
    range.color.r = 0.85; range.color.g = 0.3; range.color.b = 0.1; range.color.a = 0.7;
    for (int i = 0; i <= 72; ++i) {
      const double a = 2.0 * 3.14159265358979323846 * i / 72;
      range.points.push_back(point({pose_.position.x + config_.vision_radius * std::cos(a),
                                     pose_.position.y + config_.vision_radius * std::sin(a)}, 0.015));
    }
    markers.markers.push_back(range);
    auto rays = marker("bundles", 0, Marker::LINE_LIST, stamp);
    rays.scale.x = 0.012;
    rays.color.r = 0.0; rays.color.g = 0.65; rays.color.b = 0.9; rays.color.a = 0.7;
    for (const auto& bundle : manager_->bundle_history().bundles()) {
      for (const auto& segment : bundle.segments) {
        rays.points.push_back(point(segment.start, 0.08));
        rays.points.push_back(point(segment.end, 0.08));
      }
    }
    markers.markers.push_back(rays);
    marker_pub_->publish(markers);
  }

  mars::navigation_decision::NavigationConfig config_;
  std::unique_ptr<GraphBundleManager> manager_;
  std::unique_ptr<Navigator> navigator_;
  std::unique_ptr<mars::demo_simulation::PathFollower> follower_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  Point2D goal_{}, entry_pose_{};
  double spawn_x_{0.0}, spawn_y_{0.0};
  Pose2D pose_{};
  Pose2D progress_pose_{};
  LaserFrame scan_{};
  std::vector<Point2D> entry_path_, trail_, planned_, entry_trace_, return_trace_;
  std::vector<BlockedTarget> blocked_targets_;
  std::string current_event_;
  std::string status_log_path_;
  std::ofstream status_log_;
  std::string logged_event_;
  std::chrono::steady_clock::time_point scan_received_{}, pose_received_{};
  std::chrono::steady_clock::time_point progress_time_{};
  std::uint64_t scan_sequence_{0}, scan_used_{0}, map_scan_used_{0}, observation_id_{0};
  std::uint64_t logged_scan_sequence_{0};
  std::size_t last_open_points_{0};
  bool have_pose_{false}, active_{false}, terminal_{false};
  RecoveryPhase recovery_phase_{RecoveryPhase::None};
  std::chrono::steady_clock::time_point recovery_deadline_{};
  double recovery_target_yaw_{0.0};
  double turn_sign_{1.0};
  int recovery_attempts_{0};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trail_pub_, plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr entry_pub_, return_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  nav_msgs::msg::OccupancyGrid observed_map_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OnlineNavigationNode>());
  rclcpp::shutdown();
  return 0;
}
