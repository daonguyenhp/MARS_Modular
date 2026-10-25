#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "tf2/utils.h"
#include "visualization_msgs/MarkerArray.h"

#include "laser_geometry.hpp"
#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/navigation_decision/navigator.hpp"
#include "mars/perception_geometry/perception.hpp"
#include "path_follower.hpp"

namespace mars::online_jettank {
namespace {

using geometry_msgs::Point;
using geometry_msgs::PoseStamped;
using geometry_msgs::Twist;
using mars::common::Point2D;
using mars::common::Pose2D;
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
    pnh_.param("arrival_tolerance", config_.arrival_tolerance, 0.20);
    pnh_.param("goal_tolerance", config_.goal_tolerance, 0.25);
    pnh_.param("linear_epsilon", config_.linear_epsilon, 0.03);
    pnh_.param("angular_epsilon", config_.angular_epsilon, 0.03);
    pnh_.param("goal_x", goal_.x, 8.6);
    pnh_.param("goal_y", goal_.y, 6.8);

    mars::graph_bundle_management::GraphBundleConfig bundle_cfg;
    bundle_cfg.linear_epsilon = config_.linear_epsilon;
    bundle_cfg.angular_epsilon = config_.angular_epsilon;
    bundle_cfg.visited_radius = config_.arrival_tolerance;
    bundle_cfg.gate_contract_enabled = true;
    manager_ = std::make_unique<GraphBundleManager>(bundle_cfg);
    navigator_ = std::make_unique<Navigator>(config_);

    mars::demo_simulation::FollowerConfig follower_cfg;
    follower_cfg.max_linear_speed = 0.25;  // JetTank safe speed in m/s
    follower_cfg.max_angular_speed = 0.8; // rad/s
    follower_cfg.lookahead = 0.25;
    follower_cfg.arrival_tolerance = config_.arrival_tolerance;
    follower_ = std::make_unique<mars::demo_simulation::PathFollower>(follower_cfg);

    init_occupancy_grid();

    std::string scan_topic = "scan_raw";
    pnh_.param("scan_topic", scan_topic, scan_topic);
    scan_sub_ = nh_.subscribe(scan_topic, 5, &OnlineJetTankNode::on_scan, this);
    if (scan_topic != "scan") {
      scan_sub_fallback_ = nh_.subscribe("scan", 5, &OnlineJetTankNode::on_scan, this);
    }
    odom_sub_ = nh_.subscribe("odom", 10, &OnlineJetTankNode::on_odom, this);
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
    ROS_INFO("MARS Online JetTank node initialized. Goal: (%.2f, %.2f), Vision: %.2fm",
             goal_.x, goal_.y, config_.vision_radius);
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

  void on_odom(const Odometry::ConstPtr& msg) {
    pose_.position.x = msg->pose.pose.position.x;
    pose_.position.y = msg->pose.pose.position.y;
    pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);
    if (!have_pose_) {
      have_pose_ = true;
      entry_pose_ = pose_.position;
      progress_pose_ = pose_;
      progress_time_ = ros::Time::now();
      trail_.push_back(pose_.position);
      entry_path_.push_back(pose_.position);
      ROS_INFO("First odometry received at (%.2f, %.2f, yaw=%.2f)",
               pose_.position.x, pose_.position.y, pose_.yaw);
    }
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
      const auto command = follower_->compute(pose_);
      if (follower_->path_done()) {
        stop_robot();
        active_ = false;
        if (current_event_ == "EXPLORE" || current_event_ == "GOAL_VISIBLE") {
          entry_path_.push_back(pose_.position);
          entry_trace_.push_back(pose_.position);
        } else if (current_event_ == "FUNNEL") {
          return_trace_.push_back(pose_.position);
        }
      } else {
        if (distance(pose_.position, progress_pose_.position) > 0.03 ||
            std::abs(std::atan2(std::sin(pose_.yaw - progress_pose_.yaw),
                                std::cos(pose_.yaw - progress_pose_.yaw))) > 0.15) {
          progress_pose_ = pose_;
          progress_time_ = ros::Time::now();
        }
        if ((ros::Time::now() - progress_time_).toSec() > 15.0) {
          stop_robot();
          active_ = false;
          terminal_ = true;
          current_event_ = "MOTION_STALLED";
          ROS_ERROR("JetTank stalled: cannot follow selected path");
          return;
        }

        // Safety collision check: JetTank front clearance
        if (command.linear_x > 0 && forward_clearance(scan_) < 0.22) {
          stop_robot();
          active_ = false;
          current_event_ = ++safety_stops_ >= 3 ? "SAFETY_BLOCKED" : "SAFETY_STOP";
          if (safety_stops_ >= 3) terminal_ = true;
          ROS_WARN("Safety stop triggered! Clearance: %.2fm", forward_clearance(scan_));
        } else {
          if (command.linear_x > 0) safety_stops_ = 0;
          Twist twist;
          twist.linear.x = command.linear_x;
          twist.angular.z = command.angular_z;
          cmd_pub_.publish(twist);
          jettank_cmd_pub_.publish(twist);
          if (trail_.empty() || distance(trail_.back(), pose_.position) > 0.05) {
            trail_.push_back(pose_.position);
          }
        }
      }
    } else if (scan_sequence_ > scan_used_) {
      plan_from_current_scan();
      scan_used_ = scan_sequence_;
    }
  }

  void plan_from_current_scan() {
    const auto obstacles = scan_to_obstacles(scan_, pose_, config_.vision_radius);
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
    observation.goal_visible = target_visible_in_scan(
        scan_, pose_, goal_, config_.vision_radius);
    observation.entry_pose = entry_pose_;
    observation.entry_path = entry_path_;
    observation.ranked = manager_->ranked_open_points();
    if (navigator_->return_target()) {
      observation.at_return_target =
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
                if (distance(visited, pose_.position) < 1e-9) continue;
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
      decision = navigator_->decide(observation); // transition to FUNNEL immediately
    }
    current_event_ = decision.event;
    planned_ = decision.planned_path.points;

    ROS_INFO("Observation %zu: event=%s, planned points=%zu",
             static_cast<std::size_t>(observation_id_),
             current_event_.c_str(), planned_.size());

    if (decision.state == mars::navigation_decision::NavigationState::GoalReached ||
        decision.state == mars::navigation_decision::NavigationState::Failed ||
        decision.event == "FUNNEL_FAIL") {
      terminal_ = true;
      stop_robot();
      return;
    }
    if (decision.event == "RETURN_REACHED") return;
    if (planned_.size() < 2) {
      current_event_ = "NO_VALID_PATH";
      terminal_ = true;
      stop_robot();
      return;
    }

    follower_->set_path(planned_);
    progress_pose_ = pose_;
    progress_time_ = ros::Time::now();
    active_ = true;
  }

  void publish_visuals() {
    const ros::Time stamp = ros::Time::now();
    String status_msg;
    status_msg.data = current_event_;
    status_pub_.publish(status_msg);

    auto make_path = [&](const std::vector<Point2D>& points) {
      Path p;
      p.header.frame_id = "odom";
      p.header.stamp = stamp;
      for (const auto pt : points) {
        PoseStamped ps;
        ps.header = p.header;
        ps.pose.position.x = pt.x;
        ps.pose.position.y = pt.y;
        ps.pose.orientation.w = 1.0;
        p.poses.push_back(ps);
      }
      return p;
    };
    trail_pub_.publish(make_path(trail_));
    plan_pub_.publish(make_path(planned_));
    entry_pub_.publish(make_path(entry_trace_));
    return_pub_.publish(make_path(return_trace_));

    MarkerArray markers;
    Marker clear = make_marker("clear", 0, Marker::LINE_LIST, stamp);
    clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);

    // Goal marker
    Marker goal_m = make_marker("goal", 0, Marker::SPHERE, stamp);
    goal_m.pose.position = to_ros_point(goal_, 0.12);
    goal_m.scale.x = 0.25; goal_m.scale.y = 0.25; goal_m.scale.z = 0.25;
    goal_m.color.r = 0.95; goal_m.color.g = 0.15; goal_m.color.b = 0.10; goal_m.color.a = 1.0;
    markers.markers.push_back(goal_m);

    // Vision range circle
    Marker range = make_marker("vision", 0, Marker::LINE_STRIP, stamp);
    range.scale.x = 0.02;
    range.color.r = 0.85; range.color.g = 0.30; range.color.b = 0.10; range.color.a = 0.7;
    for (int i = 0; i <= 72; ++i) {
      const double a = 2.0 * M_PI * i / 72.0;
      range.points.push_back(to_ros_point({pose_.position.x + config_.vision_radius * std::cos(a),
                                           pose_.position.y + config_.vision_radius * std::sin(a)}, 0.02));
    }
    markers.markers.push_back(range);

    // Bundle rays
    Marker rays = make_marker("bundles", 0, Marker::LINE_LIST, stamp);
    rays.scale.x = 0.015;
    rays.color.r = 0.0; rays.color.g = 0.65; rays.color.b = 0.90; rays.color.a = 0.7;
    for (const auto& bundle : manager_->bundle_history().bundles()) {
      for (const auto& segment : bundle.segments) {
        rays.points.push_back(to_ros_point(segment.start, 0.08));
        rays.points.push_back(to_ros_point(segment.end, 0.08));
      }
    }
    markers.markers.push_back(rays);
    marker_pub_.publish(markers);
  }

  ros::NodeHandle nh_, pnh_;
  mars::navigation_decision::NavigationConfig config_;
  std::unique_ptr<GraphBundleManager> manager_;
  std::unique_ptr<Navigator> navigator_;
  std::unique_ptr<mars::demo_simulation::PathFollower> follower_;

  Point2D goal_{}, entry_pose_{};
  Pose2D pose_{}, progress_pose_{};
  LaserFrame scan_{};
  std::vector<Point2D> entry_path_, trail_, planned_, entry_trace_, return_trace_;
  std::string current_event_{"INIT"};
  ros::Time progress_time_{}, scan_time_{};
  std::uint64_t scan_sequence_{0}, scan_used_{0}, observation_id_{0};
  bool have_pose_{false}, active_{false}, terminal_{false};
  int safety_stops_{0};

  ros::Subscriber scan_sub_, scan_sub_fallback_, odom_sub_, goal_sub_;
  ros::Publisher cmd_pub_, jettank_cmd_pub_, status_pub_, marker_pub_;
  ros::Publisher trail_pub_, plan_pub_, entry_pub_, return_pub_, map_pub_;
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
