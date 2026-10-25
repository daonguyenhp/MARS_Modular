#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/MarkerArray.h"

#include "laser_geometry.hpp"
#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/graph_bundle_management/shared_sight_connector.hpp"
#include "mars/perception_geometry/perception_pipeline.hpp"
#include "navigator.hpp"
#include "path_follower.hpp"

namespace mars::online_jettank {
namespace {

using geometry_msgs::Point;
using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using mars::common::Point2D;
using mars::common::Pose2D;
using mars::common::Polygon2D;
using mars::graph_bundle_management::GraphBundleManager;
using mars::navigation_decision::NavigationObservation;
using mars::navigation_decision::Navigator;
using nav_msgs::OccupancyGrid;
using nav_msgs::Odometry;
using nav_msgs::Path;
using sensor_msgs::LaserScan;
using std_msgs::String;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

double distance(Point2D a, Point2D b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

Point to_ros_point(Point2D p, double z = 0.0) {
  Point out;
  out.x = p.x;
  out.y = p.y;
  out.z = z;
  return out;
}

Marker make_marker(const std::string& ns, int id, int type, const ros::Time& stamp) {
  Marker m;
  m.header.frame_id = "odom";
  m.header.stamp = stamp;
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  return m;
}

}  // namespace

class OnlineJetTankNode {
 public:
  explicit OnlineJetTankNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh), pnh_(pnh) {
    pnh_.param("vision_radius", config_.vision_radius, 1.6);
    pnh_.param("max_step", config_.max_step, 2.0);
    pnh_.param("arrival_tolerance", config_.arrival_tolerance, 0.15);
    pnh_.param("goal_tolerance", config_.goal_tolerance, 0.12);
    pnh_.param("linear_epsilon", config_.linear_epsilon, 0.03);
    pnh_.param("goal_x", goal_.x, 8.6);
    pnh_.param("goal_y", goal_.y, 8.6);
    pnh_.param("robot_name", robot_name_, std::string("jettank"));

    mars::graph_bundle_management::GraphBundleConfig bundle_cfg;
    bundle_cfg.linear_epsilon = config_.linear_epsilon;
    bundle_cfg.visited_radius = config_.arrival_tolerance;
    bundle_cfg.gate_contract_enabled = true;
    manager_ = std::make_unique<GraphBundleManager>(bundle_cfg);
    navigator_ = std::make_unique<Navigator>(config_);

    mars::demo_simulation::FollowerConfig follower_cfg;
    follower_cfg.max_linear_speed = 0.20;  // JetTank safe speed in m/s
    follower_cfg.max_angular_speed = 0.75; // rad/s
    follower_cfg.waypoint_tolerance = 0.10;
    follower_cfg.goal_tolerance = 0.10;
    follower_ = std::make_unique<mars::demo_simulation::PathFollower>(follower_cfg);

    init_occupancy_grid();

    std::string scan_topic = "scan_raw";
    pnh_.param("scan_topic", scan_topic, scan_topic);
    scan_sub_ = nh_.subscribe(scan_topic, 5, &OnlineJetTankNode::on_scan, this);
    if (scan_topic != "scan") {
      scan_sub_fallback_ = nh_.subscribe("scan", 5, &OnlineJetTankNode::on_scan, this);
    }
    odom_sub_ = nh_.subscribe("odom", 10, &OnlineJetTankNode::on_odom, this);
    model_states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &OnlineJetTankNode::on_model_states, this);
    goal_sub_ = nh_.subscribe("move_base_simple/goal", 1, &OnlineJetTankNode::on_goal, this);

    cmd_pub_ = nh_.advertise<Twist>("cmd_vel", 5);
    jettank_cmd_pub_ = nh_.advertise<Twist>("hiwonder_controller/cmd_vel", 5);
    status_pub_ = nh_.advertise<String>("mars_online/status", 1);
    marker_pub_ = nh_.advertise<MarkerArray>("mars_online/markers", 1);
    trail_pub_ = nh_.advertise<Path>("mars_online/trail", 1);
    plan_pub_ = nh_.advertise<Path>("mars_online/plan", 1);
    entry_pub_ = nh_.advertise<Path>("mars_online/entry", 1);
    return_pub_ = nh_.advertise<Path>("mars_online/return", 1);
    map_pub_ = nh_.advertise<OccupancyGrid>("mars_online/map", 1);

    timer_ = nh_.createTimer(ros::Duration(0.05), &OnlineJetTankNode::control_step, this);
    ROS_INFO("MARS Online JetTank node initialized. Goal: (%.2f, %.2f), Vision: %.2fm, MaxStep: %.2fm",
             goal_.x, goal_.y, config_.vision_radius, config_.max_step);
  }

 private:
  void init_occupancy_grid() {
    observed_map_.header.frame_id = "odom";
    observed_map_.info.resolution = 0.05;
    observed_map_.info.width = 240;
    observed_map_.info.height = 240;
    observed_map_.info.origin.position.x = -2.0;
    observed_map_.info.origin.position.y = -2.0;
    observed_map_.info.origin.position.z = 0.0;
    observed_map_.info.origin.orientation.w = 1.0;
    observed_map_.data.assign(observed_map_.info.width * observed_map_.info.height, -1);
  }

  void update_pose(double x, double y, double yaw) {
    pose_.position.x = x;
    pose_.position.y = y;
    pose_.yaw = yaw;
    if (!have_pose_) {
      have_pose_ = true;
      entry_pose_ = pose_.position;
      progress_pose_ = pose_;
      progress_time_ = ros::Time::now();
      trail_.push_back(pose_.position);
      entry_path_.push_back(pose_.position);
      entry_trace_.push_back(pose_.position);
      ROS_INFO("Robot initial spawn pose verified at (%.2f, %.2f, yaw=%.2f)",
               pose_.position.x, pose_.position.y, pose_.yaw);
    }

    const ros::Time now = ros::Time::now();
    if (now > last_tf_stamp_ + ros::Duration(0.02)) {
      last_tf_stamp_ = now;
      geometry_msgs::TransformStamped tf_msg;
      tf_msg.header.stamp = now;
      tf_msg.header.frame_id = "odom";
      tf_msg.child_frame_id = "base_footprint";
      tf_msg.transform.translation.x = x;
      tf_msg.transform.translation.y = y;
      tf_msg.transform.translation.z = 0.0;
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      tf_msg.transform.rotation.x = q.x();
      tf_msg.transform.rotation.y = q.y();
      tf_msg.transform.rotation.z = q.z();
      tf_msg.transform.rotation.w = q.w();
      tf_broadcaster_.sendTransform(tf_msg);
    }
  }

  void on_model_states(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == robot_name_ || msg->name[i] == "jettank") {
        update_pose(msg->pose[i].position.x, msg->pose[i].position.y,
                    tf2::getYaw(msg->pose[i].orientation));
        have_ground_truth_ = true;
        break;
      }
    }
  }

  void on_odom(const Odometry::ConstPtr& msg) {
    if (have_ground_truth_) return;
    update_pose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                tf2::getYaw(msg->pose.pose.orientation));
  }

  void on_scan(const LaserScan::ConstPtr& msg) {
    scan_.ranges = msg->ranges;
    scan_.angle_min = msg->angle_min;
    scan_.angle_increment = msg->angle_increment;
    scan_.range_min = msg->range_min;
    scan_.range_max = msg->range_max;
    ++scan_sequence_;
    scan_time_ = msg->header.stamp;
  }

  void on_goal(const PoseStamped::ConstPtr& msg) {
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    terminal_ = false;
    active_ = false;
    current_event_ = "NEW_GOAL";
    ROS_INFO("New goal set to (%.2f, %.2f)", goal_.x, goal_.y);
  }

  void stop_robot() {
    Twist zero;
    cmd_pub_.publish(zero);
    jettank_cmd_pub_.publish(zero);
  }

  struct TrackedCommand {
    double linear_x{0.0};
    double angular_z{0.0};
    bool done{false};
  };

  TrackedCommand compute_tank_step() {
    TrackedCommand cmd;
    if (planned_index_ >= planned_.size()) {
      cmd.done = true;
      return cmd;
    }

    // Skip waypoints already reached
    while (planned_index_ < planned_.size()) {
      const bool is_last = (planned_index_ + 1 == planned_.size());
      const double tol = is_last ? config_.goal_tolerance : config_.arrival_tolerance;
      if (distance(pose_.position, planned_[planned_index_]) > tol) {
        break;
      }
      ++planned_index_;
    }

    if (planned_index_ >= planned_.size()) {
      cmd.done = true;
      return cmd;
    }

    const Point2D target = planned_[planned_index_];
    const double dx = target.x - pose_.position.x;
    const double dy = target.y - pose_.position.y;
    const double gap = std::hypot(dx, dy);
    const double target_yaw = std::atan2(dy, dx);
    const double yaw_error = std::atan2(std::sin(target_yaw - pose_.yaw),
                                        std::cos(target_yaw - pose_.yaw));

    // Tracked tank (bánh xe tăng) skid-steer behavior:
    // 1. Pivot in place until aligned within ~5 degrees (0.08 rad)
    if (std::abs(yaw_error) > (is_pivoting_ ? 0.08 : 0.20)) {
      is_pivoting_ = true;
      cmd.linear_x = 0.0;
      const double raw_w = 1.8 * yaw_error;
      const double min_turn_w = 0.28; // overcomes track static friction in Gazebo
      const double max_turn_w = 0.75;
      const double mag_w = std::clamp(std::abs(raw_w), min_turn_w, max_turn_w);
      cmd.angular_z = std::copysign(mag_w, yaw_error);
    } else {
      // 2. Aligned! Drive straight forward along ray with fine angular correction
      is_pivoting_ = false;
      const double max_v = 0.22;
      cmd.linear_x = std::clamp(gap, 0.06, max_v);
      cmd.angular_z = 1.2 * yaw_error;
    }
    return cmd;
  }

  void control_step(const ros::TimerEvent&) {
    if (!have_pose_ || scan_.ranges.empty()) {
      return;
    }
    publish_visuals();
    if (terminal_) {
      stop_robot();
      return;
    }

    if (active_) {
      const auto command = compute_tank_step();
      if (command.done) {
        stop_robot();
        active_ = false;
        settle_time_ = ros::Time::now() + ros::Duration(0.35); // settle before scanning next center
        if (current_event_ == "EXPLORE" || current_event_ == "GOAL_VISIBLE") {
          entry_path_.push_back(pose_.position);
          entry_trace_.push_back(pose_.position);
        } else if (current_event_ == "FUNNEL") {
          return_trace_.push_back(pose_.position);
          if (return_index_ > 0) {
            --return_index_;
          }
        }
      } else {
        if (distance(pose_.position, progress_pose_.position) > 0.03 ||
            std::abs(std::atan2(std::sin(pose_.yaw - progress_pose_.yaw),
                                std::cos(pose_.yaw - progress_pose_.yaw))) > 0.15) {
          progress_pose_ = pose_;
          progress_time_ = ros::Time::now();
        }
        if ((ros::Time::now() - progress_time_).toSec() > 20.0) {
          stop_robot();
          active_ = false;
          terminal_ = true;
          current_event_ = "MOTION_STALLED";
          ROS_ERROR("JetTank stalled: cannot follow selected path");
          return;
        }

        Twist twist;
        twist.linear.x = command.linear_x;
        twist.angular.z = command.angular_z;
        cmd_pub_.publish(twist);
        jettank_cmd_pub_.publish(twist);
        if (trail_.empty() || distance(trail_.back(), pose_.position) > 0.02) {
          trail_.push_back(pose_.position);
          if (current_event_ == "FUNNEL" || current_event_ == "RETURN_REACHED" || current_event_ == "FUNNEL_FAIL") {
            if (return_trace_.empty() && !entry_trace_.empty()) {
              return_trace_.push_back(entry_trace_.back());
            }
            return_trace_.push_back(pose_.position);
          } else {
            entry_trace_.push_back(pose_.position);
          }
        }
      }
    } else if (ros::Time::now() >= settle_time_ && scan_sequence_ > scan_used_) {
      try {
        plan_from_current_scan();
      } catch (const std::exception& e) {
        ROS_WARN("Planner exception on frame: %s (retrying on next scan)", e.what());
      }
      scan_used_ = scan_sequence_;
    }
  }

  void plan_from_current_scan() {
    latest_obstacles_ = scan_to_obstacles(scan_, pose_, config_.vision_radius);
    latest_perception_ = mars::perception_geometry::perceive(
        pose_.position, config_.vision_radius, latest_obstacles_);

    auto point_to_segment_dist = [](Point2D p, Point2D a, Point2D b) {
      const double dx = b.x - a.x;
      const double dy = b.y - a.y;
      const double len_sq = dx * dx + dy * dy;
      if (len_sq < 1e-9) return distance(p, a);
      const double t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq, 0.0, 1.0);
      const Point2D proj{a.x + t * dx, a.y + t * dy};
      return distance(p, proj);
    };

    auto cross_2d = [](Point2D a, Point2D b, Point2D c) {
      return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    };
    auto segments_intersect = [&](Point2D p1, Point2D p2, Point2D p3, Point2D p4) {
      const double d1 = cross_2d(p3, p4, p1);
      const double d2 = cross_2d(p3, p4, p2);
      const double d3 = cross_2d(p1, p2, p3);
      const double d4 = cross_2d(p1, p2, p4);
      if (((d1 > 1e-5 && d2 < -1e-5) || (d1 < -1e-5 && d2 > 1e-5)) &&
          ((d3 > 1e-5 && d4 < -1e-5) || (d3 < -1e-5 && d4 > 1e-5))) {
        return true;
      }
      return false;
    };

    // Sanitize open points:
    // 1. Must satisfy neighbor_sight_supports_point
    // 2. Direct line-of-sight must NOT be occluded by raw laser scan returns at 25%, 50%, 75%, 100% of hop
    // 3. Line from robot to candidate point must NOT cross any perceived obstacle wall segment
    // 4. Candidate open point and entire path must have safe clearance from all obstacle walls
    latest_perception_.open_points.erase(
        std::remove_if(latest_perception_.open_points.begin(), latest_perception_.open_points.end(),
                       [&](const mars::perception_geometry::OpenPoint& op) {
                         if (!mars::graph_bundle_management::neighbor_sight_supports_point(
                                 latest_perception_.neighbor_sight, op.point, config_.linear_epsilon)) {
                           return true;
                         }
                         for (double frac = 0.25; frac <= 1.0; frac += 0.25) {
                           const Point2D sample{pose_.position.x + frac * (op.point.x - pose_.position.x),
                                                pose_.position.y + frac * (op.point.y - pose_.position.y)};
                           if (is_point_occluded_in_scan(scan_, pose_, sample, 0.04)) {
                             return true;
                           }
                           for (const auto& poly : latest_obstacles_) {
                             for (std::size_t vi = 0; vi + 1 < poly.vertices.size(); ++vi) {
                               const Point2D w1 = poly.vertices[vi];
                               const Point2D w2 = poly.vertices[vi + 1];
                               if (segments_intersect(pose_.position, op.point, w1, w2)) {
                                 return true;
                               }
                               if (point_to_segment_dist(sample, w1, w2) < 0.18) {
                                 return true;
                               }
                             }
                           }
                         }
                         for (const auto& poly : latest_obstacles_) {
                           for (std::size_t vi = 0; vi + 1 < poly.vertices.size(); ++vi) {
                             if (point_to_segment_dist(op.point, poly.vertices[vi], poly.vertices[vi + 1]) < 0.22) {
                               return true;
                             }
                           }
                         }
                         return false;
                       }),
        latest_perception_.open_points.end());

    mars::graph_bundle_management::GraphBundleUpdate update;
    update.observation_id = ++observation_id_;
    update.current_pose = pose_;
    update.goal = goal_;
    update.perception = latest_perception_;
    manager_->update(update);

    NavigationObservation observation;
    observation.pose = pose_.position;
    observation.goal = goal_;
    observation.goal_visible = target_visible_in_scan(
        scan_, pose_, goal_, config_.vision_radius);
    observation.entry_pose = entry_pose_;
    observation.entry_path = entry_path_;
    observation.ranked = manager_->ranked_open_points();
    if (navigator_->return_target()) {
      observation.at_return_target =
          (return_index_ == 0) ||
          distance(pose_.position, *navigator_->return_target()) <= config_.arrival_tolerance;
    }

    for (const auto& ranked : observation.ranked) {
      mars::navigation_decision::EscapeCandidate candidate;
      candidate.id = ranked.id;
      candidate.point = ranked.point;
      candidate.score = ranked.score;
      candidate.owner_pose = pose_.position;
      if (const auto* record = manager_->open_point_memory().find(ranked.id)) {
        candidate.owner_pose = record->source_center;
      }
      observation.escape_candidates.push_back(candidate);

      const auto prepared = manager_->prepare_route(ranked.id);
      if (!prepared.success) continue;
      mars::navigation_decision::ReachableRoute route;
      route.id = ranked.id;
      for (const auto id : prepared.skeleton_path.node_ids) {
        if (const auto* node = manager_->visibility_graph().find_node(id)) {
          route.skeleton.push_back(node->position);
        }
      }
      if (route.skeleton.size() >= 2) {
        observation.routes.push_back(std::move(route));
      }
    }

    // Filter routes returning to driven poses
    observation.routes.erase(
        std::remove_if(
            observation.routes.begin(), observation.routes.end(),
            [&](const mars::navigation_decision::ReachableRoute& route) {
              const auto steps = mars::navigation_decision::cap_last_hop(route.skeleton, config_.max_step);
              for (const auto& visited : entry_path_) {
                if (distance(visited, pose_.position) <= config_.linear_epsilon) continue;
                for (std::size_t i = 1; i < steps.size(); ++i) {
                  if (distance(visited, steps[i]) <= config_.arrival_tolerance) return true;
                }
              }
              return false;
            }),
        observation.routes.end());

    if (observation.routes.empty()) {
      observation.ranked.clear();
      // Keep escape_candidates intact so plan_escape has target!
    }
    if (const auto gates = manager_->return_gates(); gates.success) {
      observation.gates = gates.gates;
    }

    auto decision = navigator_->decide(observation);
    if (decision.event == "BAR") {
      ROS_INFO("Observation %zu: Blind Alley Detected! Transitioning to DAP Funnel...",
               static_cast<std::size_t>(observation_id_));
      if (return_index_ < 0) {
        return_index_ = static_cast<int>(entry_path_.size()) - 1;
      }
      if (return_trace_.empty()) {
        return_trace_.push_back(pose_.position);
      }
      decision = navigator_->decide(observation); // transition to FUNNEL immediately
    }
    current_event_ = decision.event;
    planned_ = decision.planned_path.points;

    // In FUNNEL mode, backtrack step-by-step safely along the sequence of observation centers:
    if (current_event_ == "FUNNEL" || current_event_ == "FUNNEL_FAIL") {
      if (return_index_ < 0 && !entry_path_.empty()) {
        return_index_ = static_cast<int>(entry_path_.size()) - 1;
      }
      if (return_trace_.empty()) {
        return_trace_.push_back(pose_.position);
      }
      if (return_index_ > 0 && return_index_ - 1 < static_cast<int>(entry_path_.size())) {
        const Point2D target_center = entry_path_[return_index_ - 1];
        planned_ = {pose_.position, target_center};
        current_event_ = "FUNNEL";
        ROS_INFO("Observation %zu: DAP funnel step to C%d (%.2f, %.2f), distance: %.2fm",
                 static_cast<std::size_t>(observation_id_),
                 return_index_ - 1, target_center.x, target_center.y,
                 distance(pose_.position, target_center));
      }
    }

    ROS_INFO("Observation %zu: event=%s, planned points=%zu, walls=%zu, open_points=%zu",
             static_cast<std::size_t>(observation_id_),
             current_event_.c_str(), planned_.size(), latest_obstacles_.size(), latest_perception_.open_points.size());
    if (planned_.size() >= 2) {
      ROS_INFO("Target waypoint: (%.2f, %.2f), distance: %.2fm",
               planned_.back().x, planned_.back().y, distance(pose_.position, planned_.back()));
    }

    if (decision.state == mars::navigation_decision::NavigationState::GoalReached ||
        decision.state == mars::navigation_decision::NavigationState::Failed) {
      terminal_ = true;
      stop_robot();
      return;
    }
    if (decision.event == "RETURN_REACHED") {
      terminal_ = true;
      stop_robot();
      return_trace_.push_back(pose_.position);
      ROS_INFO("Escape complete! JetTank reached C0 / return target.");
      return;
    }

    if (planned_.size() < 2) {
      current_event_ = "NO_VALID_PATH";
      terminal_ = true;
      stop_robot();
      return;
    }

    planned_index_ = 0;
    progress_pose_ = pose_;
    progress_time_ = ros::Time::now();
    active_ = true;
  }

  void publish_visuals() {
    const auto stamp = ros::Time::now();
    std_msgs::String status_msg;
    status_msg.data = current_event_;
    status_pub_.publish(status_msg);

    if (!have_pose_) return;

    auto publish_path = [&](const std::vector<Point2D>& points, ros::Publisher& pub, double z_elev = 0.05) {
      if (points.empty()) return;
      Path path_msg;
      path_msg.header.frame_id = "odom";
      path_msg.header.stamp = stamp;
      for (const auto p : points) {
        PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position = to_ros_point(p, z_elev);
        ps.pose.orientation.w = 1.0;
        path_msg.poses.push_back(ps);
      }
      pub.publish(path_msg);
    };

    publish_path(trail_, trail_pub_, 0.02);
    publish_path(entry_trace_, entry_pub_, 0.05);
    publish_path(return_trace_, return_pub_, 0.07);
    publish_path(planned_, plan_pub_, 0.09);

    MarkerArray markers;

    // 1. Ray bundles: clean, bounded fan lines (36 rays, exactly 10 deg spacing) in laser_frame (NO skew!)
    Marker rays = make_marker("ray_bundles", 0, Marker::LINE_LIST, stamp);
    rays.header.frame_id = "laser_frame";
    rays.scale.x = 0.012;
    rays.color.r = 0.12;
    rays.color.g = 0.47;
    rays.color.b = 0.71;
    rays.color.a = 0.50;
    if (!scan_.ranges.empty() && scan_.angle_increment > 0.0) {
      const std::size_t step = std::max<std::size_t>(1, scan_.ranges.size() / 36);
      for (std::size_t i = 0; i < scan_.ranges.size(); i += step) {
        const double r = static_cast<double>(scan_.ranges[i]);
        if (!std::isfinite(r) || r <= scan_.range_min) continue;
        const double display_r = std::min(r, config_.vision_radius);
        const double scan_angle = scan_.angle_min + static_cast<double>(i) * scan_.angle_increment;
        Point p0; p0.x = 0.0; p0.y = 0.0; p0.z = 0.0;
        Point p1;
        p1.x = display_r * std::cos(scan_angle);
        p1.y = display_r * std::sin(scan_angle);
        p1.z = 0.0;
        rays.points.push_back(p0);
        rays.points.push_back(p1);
      }
    }
    markers.markers.push_back(rays);

    // 2. Vision boundary ring (R = 1.6m) centered on laser_frame
    Marker vision_ring = make_marker("vision_boundary", 10, Marker::LINE_STRIP, stamp);
    vision_ring.header.frame_id = "laser_frame";
    vision_ring.scale.x = 0.014;
    vision_ring.color.r = 0.84;
    vision_ring.color.g = 0.15;
    vision_ring.color.b = 0.16;
    vision_ring.color.a = 0.35;
    const int ring_segments = 64;
    for (int k = 0; k <= ring_segments; ++k) {
      const double theta = (2.0 * M_PI * k) / ring_segments;
      Point p;
      p.x = config_.vision_radius * std::cos(theta);
      p.y = config_.vision_radius * std::sin(theta);
      p.z = 0.0;
      vision_ring.points.push_back(p);
    }
    markers.markers.push_back(vision_ring);

    // 3. Open Sights (vùng tia mở): soft pastel sage fan sectors
    Marker open_fans = make_marker("open_sights", 1, Marker::TRIANGLE_LIST, stamp);
    open_fans.scale.x = 1.0;
    open_fans.scale.y = 1.0;
    open_fans.scale.z = 1.0;
    open_fans.color.r = 0.50;
    open_fans.color.g = 0.82;
    open_fans.color.b = 0.55;
    open_fans.color.a = 0.15;
    for (const auto& sight : latest_perception_.open_sights) {
      const double start = sight.interval.start;
      const double sweep = sight.interval.sweep;
      const int slices = std::max(6, static_cast<int>(sweep / 0.08));
      const double step = sweep / static_cast<double>(slices);
      for (int s = 0; s < slices; ++s) {
        const double a1 = start + static_cast<double>(s) * step;
        const double a2 = start + static_cast<double>(s + 1) * step;
        const Point2D p0 = latest_perception_.neighbor_sight.center;
        const Point2D p1{p0.x + config_.vision_radius * std::cos(a1),
                         p0.y + config_.vision_radius * std::sin(a1)};
        const Point2D p2{p0.x + config_.vision_radius * std::cos(a2),
                         p0.y + config_.vision_radius * std::sin(a2)};
        open_fans.points.push_back(to_ros_point(p0, 0.04));
        open_fans.points.push_back(to_ros_point(p1, 0.04));
        open_fans.points.push_back(to_ros_point(p2, 0.04));
      }
    }
    markers.markers.push_back(open_fans);

    // 4. Closed Sights (vùng tia đóng/chặn): soft pastel rose fan sectors
    Marker closed_fans = make_marker("closed_sights", 2, Marker::TRIANGLE_LIST, stamp);
    closed_fans.scale.x = 1.0;
    closed_fans.scale.y = 1.0;
    closed_fans.scale.z = 1.0;
    closed_fans.color.r = 0.95;
    closed_fans.color.g = 0.65;
    closed_fans.color.b = 0.70;
    closed_fans.color.a = 0.12;
    for (const auto& sight : latest_perception_.closed_sights) {
      const double start = sight.interval.start;
      const double sweep = sight.interval.sweep;
      const int slices = std::max(6, static_cast<int>(sweep / 0.08));
      const double step = sweep / static_cast<double>(slices);
      for (int s = 0; s < slices; ++s) {
        const double a1 = start + static_cast<double>(s) * step;
        const double a2 = start + static_cast<double>(s + 1) * step;
        const Point2D p0 = latest_perception_.neighbor_sight.center;
        const Point2D p1{p0.x + config_.vision_radius * std::cos(a1),
                         p0.y + config_.vision_radius * std::sin(a1)};
        const Point2D p2{p0.x + config_.vision_radius * std::cos(a2),
                         p0.y + config_.vision_radius * std::sin(a2)};
        closed_fans.points.push_back(to_ros_point(p0, 0.03));
        closed_fans.points.push_back(to_ros_point(p1, 0.03));
        closed_fans.points.push_back(to_ros_point(p2, 0.03));
      }
    }
    markers.markers.push_back(closed_fans);

    // 5. Perceived Obstacle Walls: clean dark slate lines
    Marker walls = make_marker("perceived_walls", 3, Marker::LINE_LIST, stamp);
    walls.scale.x = 0.04;
    walls.color.r = 0.30;
    walls.color.g = 0.35;
    walls.color.b = 0.40;
    walls.color.a = 0.90;
    for (const auto& poly : latest_obstacles_) {
      if (poly.vertices.size() >= 2) {
        walls.points.push_back(to_ros_point(poly.vertices.front(), 0.10));
        walls.points.push_back(to_ros_point(poly.vertices.back(), 0.10));
      }
    }
    markers.markers.push_back(walls);

    // 6. Open Points: crisp gold dots on sensing circle
    Marker points = make_marker("open_points", 4, Marker::SPHERE_LIST, stamp);
    points.scale.x = 0.12;
    points.scale.y = 0.12;
    points.scale.z = 0.12;
    points.color.r = 0.95;
    points.color.g = 0.70;
    points.color.b = 0.0;
    points.color.a = 0.95;
    for (const auto& op : latest_perception_.open_points) {
      points.points.push_back(to_ros_point(op.point, 0.12));
    }
    markers.markers.push_back(points);

    // 7. Dedicated 3D Entry Path Ribbon (Đường đi vào - Màu Đỏ)
    Marker entry_marker = make_marker("entry_path_marker", 20, Marker::LINE_STRIP, stamp);
    entry_marker.scale.x = 0.06;
    entry_marker.color.r = 0.84;
    entry_marker.color.g = 0.15;
    entry_marker.color.b = 0.16;
    entry_marker.color.a = 1.0;
    for (const auto& p : entry_trace_) {
      entry_marker.points.push_back(to_ros_point(p, 0.05));
    }
    markers.markers.push_back(entry_marker);

    // 8. Dedicated 3D Return Path Ribbon (Đường đi ra - Màu Xanh lam)
    Marker return_marker = make_marker("return_path_marker", 21, Marker::LINE_STRIP, stamp);
    return_marker.scale.x = 0.07;
    return_marker.color.r = 0.12;
    return_marker.color.g = 0.47;
    return_marker.color.b = 0.85;
    return_marker.color.a = 1.0;
    for (const auto& p : return_trace_) {
      return_marker.points.push_back(to_ros_point(p, 0.07));
    }
    markers.markers.push_back(return_marker);

    // 9. Observation Centers & 3D Text Labels
    for (std::size_t i = 0; i < entry_path_.size(); ++i) {
      Marker center_pt = make_marker("observation_centers", static_cast<int>(50 + i), Marker::CYLINDER, stamp);
      center_pt.scale.x = 0.18;
      center_pt.scale.y = 0.18;
      center_pt.scale.z = 0.03;
      if (i == 0) {
        // C0 (Start)
        center_pt.color.r = 0.15;
        center_pt.color.g = 0.75;
        center_pt.color.b = 0.30;
        center_pt.color.a = 0.95;
      } else if (i + 1 == entry_path_.size() && (current_event_ == "BAR" || current_event_ == "FUNNEL" || current_event_ == "RETURN_REACHED")) {
        // C4 (BAR / Blind Alley Root - Đáy hẻm)
        center_pt.color.r = 0.84;
        center_pt.color.g = 0.15;
        center_pt.color.b = 0.16;
        center_pt.color.a = 0.95;
      } else {
        // Intermediate exploration centers
        center_pt.color.r = 0.90;
        center_pt.color.g = 0.92;
        center_pt.color.b = 0.95;
        center_pt.color.a = 0.90;
      }
      center_pt.pose.position = to_ros_point(entry_path_[i], 0.10);
      markers.markers.push_back(center_pt);

      // Observation Center 3D Text Labels
      Marker label = make_marker("center_labels", static_cast<int>(100 + i), Marker::TEXT_VIEW_FACING, stamp);
      label.scale.z = 0.22;
      label.color.r = 0.95;
      label.color.g = 0.95;
      label.color.b = 0.95;
      label.color.a = 1.0;
      label.pose.position = to_ros_point(entry_path_[i], 0.35);
      if (i == 0) {
        label.text = "C0 (Start)";
      } else if (i + 1 == entry_path_.size() && (current_event_ == "BAR" || current_event_ == "FUNNEL" || current_event_ == "RETURN_REACHED")) {
        label.text = "C" + std::to_string(i) + " (BAR)";
      } else {
        label.text = "C" + std::to_string(i);
      }
      markers.markers.push_back(label);
    }

    // 10. Goal target marker: large green sphere
    Marker target_marker = make_marker("target", 6, Marker::SPHERE, stamp);
    target_marker.scale.x = 0.26;
    target_marker.scale.y = 0.26;
    target_marker.scale.z = 0.26;
    target_marker.color.r = 0.0;
    target_marker.color.g = 1.0;
    target_marker.color.b = 0.3;
    target_marker.color.a = 0.95;
    target_marker.pose.position = to_ros_point(goal_, 0.12);
    markers.markers.push_back(target_marker);

    // 11. Active Waypoint: bright yellow cylinder
    if (planned_index_ < planned_.size()) {
      Marker wp = make_marker("active_waypoint", 7, Marker::CYLINDER, stamp);
      wp.scale.x = 0.18;
      wp.scale.y = 0.18;
      wp.scale.z = 0.30;
      wp.color.r = 1.0;
      wp.color.g = 1.0;
      wp.color.b = 0.0;
      wp.color.a = 0.90;
      wp.pose.position = to_ros_point(planned_[planned_index_], 0.20);
      markers.markers.push_back(wp);
    }

    marker_pub_.publish(markers);
  }

  ros::NodeHandle nh_, pnh_;
  mars::navigation_decision::NavigationConfig config_;
  std::unique_ptr<GraphBundleManager> manager_;
  std::unique_ptr<Navigator> navigator_;
  std::unique_ptr<mars::demo_simulation::PathFollower> follower_;

  std::string robot_name_{"jettank"};
  bool have_ground_truth_{false};
  Point2D goal_{}, entry_pose_{};
  Pose2D pose_{}, progress_pose_{};
  LaserFrame scan_{};
  mars::perception_geometry::PerceptionResult latest_perception_;
  std::vector<Polygon2D> latest_obstacles_;
  std::vector<Point2D> entry_path_, trail_, planned_, entry_trace_, return_trace_;
  std::string current_event_{"INIT"};
  ros::Time progress_time_{}, scan_time_{}, last_tf_stamp_{}, settle_time_{};
  std::uint64_t scan_sequence_{0}, scan_used_{0}, observation_id_{0};
  std::size_t planned_index_{0};
  bool have_pose_{false}, active_{false}, terminal_{false}, is_pivoting_{false};
  int safety_stops_{0}, return_index_{-1};

  ros::Subscriber scan_sub_, scan_sub_fallback_, odom_sub_, model_states_sub_, goal_sub_;
  ros::Publisher cmd_pub_, jettank_cmd_pub_, status_pub_, marker_pub_;
  ros::Publisher trail_pub_, plan_pub_, entry_pub_, return_pub_, map_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  OccupancyGrid observed_map_;
  ros::Timer timer_;
};

}  // namespace mars::online_jettank

int main(int argc, char** argv) {
  ros::init(argc, argv, "mars_online_jettank_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  mars::online_jettank::OnlineJetTankNode node(nh, pnh);
  ros::spin();
  return 0;
}
