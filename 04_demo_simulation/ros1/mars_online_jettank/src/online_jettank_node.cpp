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
        if (trail_.empty() || distance(trail_.back(), pose_.position) > 0.03) {
          trail_.push_back(pose_.position);
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
    const auto obstacles = scan_to_obstacles(scan_, pose_, config_.vision_radius);
    auto perception = mars::perception_geometry::perceive(
        pose_.position, config_.vision_radius, obstacles);

    // Sanitize open points so none violate neighbor_sight_supports_point
    perception.open_points.erase(
        std::remove_if(perception.open_points.begin(), perception.open_points.end(),
                       [&](const mars::perception_geometry::OpenPoint& op) {
                         return !mars::graph_bundle_management::neighbor_sight_supports_point(
                             perception.neighbor_sight, op.point, config_.linear_epsilon);
                       }),
        perception.open_points.end());

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
      decision = navigator_->decide(observation); // transition to FUNNEL immediately
    }
    current_event_ = decision.event;
    planned_ = decision.planned_path.points;

    ROS_INFO("Observation %zu: event=%s, planned points=%zu, walls=%zu, open_points=%zu",
             static_cast<std::size_t>(observation_id_),
             current_event_.c_str(), planned_.size(), obstacles.size(), perception.open_points.size());
    if (planned_.size() >= 2) {
      ROS_INFO("Target waypoint: (%.2f, %.2f), distance: %.2fm",
               planned_.back().x, planned_.back().y, distance(pose_.position, planned_.back()));
    }

    if (decision.state == mars::navigation_decision::NavigationState::GoalReached ||
        decision.state == mars::navigation_decision::NavigationState::Failed ||
        decision.event == "FUNNEL_FAIL") {
      terminal_ = true;
      stop_robot();
      return;
    }
    if (decision.event == "RETURN_REACHED") {
      terminal_ = true;
      stop_robot();
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

    auto publish_path = [&](const std::vector<Point2D>& points, ros::Publisher& pub) {
      Path path_msg;
      path_msg.header.frame_id = "odom";
      path_msg.header.stamp = stamp;
      for (const auto p : points) {
        PoseStamped ps;
        ps.header = path_msg.header;
        ps.pose.position = to_ros_point(p, 0.02);
        ps.pose.orientation.w = 1.0;
        path_msg.poses.push_back(ps);
      }
      pub.publish(path_msg);
    };

    publish_path(trail_, trail_pub_);
    publish_path(entry_trace_, entry_pub_);
    publish_path(return_trace_, return_pub_);
    publish_path(planned_, plan_pub_);

    // Markers for bundles and open points
    MarkerArray markers;
    Marker points = make_marker("open_points", 0, Marker::SPHERE_LIST, stamp);
    points.scale.x = 0.12;
    points.scale.y = 0.12;
    points.scale.z = 0.12;
    points.color.r = 1.0;
    points.color.g = 0.8;
    points.color.b = 0.0;
    points.color.a = 0.9;

    for (const auto& ranked : manager_->ranked_open_points()) {
      points.points.push_back(to_ros_point(ranked.point, 0.05));
    }
    markers.markers.push_back(points);

    Marker target_marker = make_marker("target", 1, Marker::SPHERE, stamp);
    target_marker.scale.x = 0.20;
    target_marker.scale.y = 0.20;
    target_marker.scale.z = 0.20;
    target_marker.color.r = 0.0;
    target_marker.color.g = 1.0;
    target_marker.color.b = 0.3;
    target_marker.color.a = 0.9;
    target_marker.pose.position = to_ros_point(goal_, 0.05);
    markers.markers.push_back(target_marker);

    Marker rays = make_marker("bundles", 2, Marker::LINE_LIST, stamp);
    rays.scale.x = 0.02;
    rays.color.r = 0.4;
    rays.color.g = 0.8;
    rays.color.b = 1.0;
    rays.color.a = 0.6;
    for (const auto& bundle : manager_->bundle_history().bundles()) {
      for (const auto& segment : bundle.segments) {
        rays.points.push_back(to_ros_point(segment.start, 0.04));
        rays.points.push_back(to_ros_point(segment.end, 0.04));
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

  std::string robot_name_{"jettank"};
  bool have_ground_truth_{false};
  Point2D goal_{}, entry_pose_{};
  Pose2D pose_{}, progress_pose_{};
  LaserFrame scan_{};
  std::vector<Point2D> entry_path_, trail_, planned_, entry_trace_, return_trace_;
  std::string current_event_{"INIT"};
  ros::Time progress_time_{}, scan_time_{}, last_tf_stamp_{}, settle_time_{};
  std::uint64_t scan_sequence_{0}, scan_used_{0}, observation_id_{0};
  std::size_t planned_index_{0};
  bool have_pose_{false}, active_{false}, terminal_{false}, is_pivoting_{false};
  int safety_stops_{0};

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
