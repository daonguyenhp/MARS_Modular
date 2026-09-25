#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/ColorRGBA.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"

#include "mode1_map.hpp"
#include "mode1_mission.hpp"

namespace {

using geometry_msgs::Point;
using std_msgs::ColorRGBA;
using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

ColorRGBA rgba(float r, float g, float b, float a) {
  ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

Point to_point(mars::common::Point2D point, double z) {
  Point out;
  out.x = point.x;
  out.y = point.y;
  out.z = z;
  return out;
}

Marker make_marker(const ros::Time& stamp, const std::string& ns, int id, int type) {
  Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = stamp;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  return marker;
}

void add_wedge(Marker& marker, mars::common::Point2D center, double radius,
               double start, double sweep) {
  const int steps = std::max(2, static_cast<int>(std::abs(sweep) / 0.12));
  for (int i = 0; i < steps; ++i) {
    const double a0 = start + sweep * (static_cast<double>(i) / steps);
    const double a1 = start + sweep * (static_cast<double>(i + 1) / steps);
    marker.points.push_back(to_point(center, 0.02));
    marker.points.push_back(to_point(
        {center.x + radius * std::cos(a0), center.y + radius * std::sin(a0)},
        0.02));
    marker.points.push_back(to_point(
        {center.x + radius * std::cos(a1), center.y + radius * std::sin(a1)},
        0.02));
  }
}

class Mode1RvizNode {
 public:
  explicit Mode1RvizNode(ros::NodeHandle& node) : node_(node) {
    map_pub_ = node_.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    marker_pub_ =
        node_.advertise<MarkerArray>("/dsfm_online/polygon_markers", 1, true);
    path_pub_ = node_.advertise<nav_msgs::Path>("/planned_path", 1, true);
    pose_pub_ =
        node_.advertise<geometry_msgs::PoseStamped>("/robot_current_pose", 1, true);
    status_pub_ = node_.advertise<std_msgs::String>("/dsfm_online/status", 1, true);

    ros::NodeHandle private_node("~");
    std::string map_id = "hard_alley";
#ifndef MARS_MODE1_MAP_DIR
#define MARS_MODE1_MAP_DIR ""
#endif
    std::string map_dir = MARS_MODE1_MAP_DIR;
    double hold_sec = 2.0;
    double drive_speed = 0.12;
    private_node.param("map_id", map_id, map_id);
    private_node.param("map_dir", map_dir, map_dir);
    private_node.param("step_hold_sec", hold_sec, hold_sec);
    private_node.param("drive_speed", drive_speed, drive_speed);

    scenario_ = mars::demo_simulation::mode1_scenario(map_id);
    map_ = mars::demo_simulation::load_occupancy_map(
        map_dir + "/" + scenario_.occupancy_yaml);
    ROS_INFO("Mode 1 map %s, obstacles=%zu", map_id.c_str(), map_.obstacles.size());
    mission_ = mars::demo_simulation::run_mode1_mission(scenario_, map_);
    ROS_INFO("stop=%s frames=%zu entry=%.2f return=%.2f",
             mission_.stop_reason.c_str(), mission_.frames.size(),
             mission_.entry_length, mission_.return_length);

    hold_sec = std::max(0.4, hold_sec);
    drive_speed_ = std::max(0.02, drive_speed);
    hold_ticks_ = std::max(1, static_cast<int>(hold_sec * kTickHz));
    begin_hold(0);
    timer_ = node_.createTimer(ros::Duration(1.0 / kTickHz),
                               &Mode1RvizNode::publish_tick, this);
  }

 private:
  void begin_hold(std::size_t frame) {
    if (mission_.frames.empty()) {
      frame_index_ = 0;
      trail_at_ = 0;
      hold_left_ = hold_ticks_;
      return;
    }
    frame_index_ = frame % mission_.frames.size();
    trail_at_ = std::min(mission_.frames[frame_index_].trail_index,
                         mission_.trail.empty() ? 0 : mission_.trail.size() - 1);
    hold_left_ = hold_ticks_;
  }

  void advance_step() {
    if (mission_.frames.size() < 2 || mission_.trail.size() < 2) {
      begin_hold(0);
      return;
    }
    const std::size_t next = frame_index_ + 1;
    if (next >= mission_.frames.size()) {
      begin_hold(0);
      return;
    }
    const std::size_t target = std::min(mission_.frames[next].trail_index,
                                        mission_.trail.size() - 1);
    if (trail_at_ >= target) {
      begin_hold(next);
      return;
    }
    double budget = drive_speed_ / kTickHz;
    while (trail_at_ < target && budget > 1.0e-4) {
      const auto& from = mission_.trail[trail_at_];
      const auto& to = mission_.trail[trail_at_ + 1];
      const double segment = std::hypot(to.x - from.x, to.y - from.y);
      if (segment > budget && segment > 1.0e-6) {
        break;
      }
      budget -= segment;
      ++trail_at_;
    }
    if (trail_at_ >= target) {
      begin_hold(next);
    }
  }

  void publish_tick(const ros::TimerEvent&) {
    const ros::Time stamp = ros::Time::now();
    publish_map(stamp);
    publish_markers(stamp);
    publish_pose_and_path(stamp);
    if (hold_left_ > 0) {
      --hold_left_;
      return;
    }
    advance_step();
  }

  void publish_map(const ros::Time& stamp) {
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = stamp;
    grid.info.resolution = static_cast<float>(map_.resolution);
    grid.info.width = static_cast<uint32_t>(map_.width);
    grid.info.height = static_cast<uint32_t>(map_.height);
    grid.info.origin.position.x = map_.origin.x;
    grid.info.origin.position.y = map_.origin.y;
    grid.info.origin.orientation.w = 1.0;
    const auto cells = static_cast<std::size_t>(map_.width) *
                       static_cast<std::size_t>(map_.height);
    grid.data.assign(cells, 0);
    for (int row = 0; row < map_.height; ++row) {
      const int ros_row = map_.height - 1 - row;
      for (int col = 0; col < map_.width; ++col) {
        const auto src = static_cast<std::size_t>(row * map_.width + col);
        const auto dst = static_cast<std::size_t>(ros_row * map_.width + col);
        if (src < map_.occupancy.size()) {
          grid.data[dst] = map_.occupancy[src];
        }
      }
    }
    map_pub_.publish(grid);
  }

  void publish_markers(const ros::Time& stamp) {
    MarkerArray array;
    Marker clear = make_marker(stamp, "", 0, Marker::DELETEALL);
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    Marker walls = make_marker(stamp, "walls", 0, Marker::TRIANGLE_LIST);
    walls.color = rgba(0.22f, 0.24f, 0.27f, 1.0f);
    walls.scale.x = walls.scale.y = walls.scale.z = 1.0f;
    for (const auto& obstacle : map_.obstacles) {
      const auto& vertices = obstacle.vertices;
      if (vertices.size() < 3) {
        continue;
      }
      for (std::size_t i = 1; i + 1 < vertices.size(); ++i) {
        walls.points.push_back(to_point(vertices[0], 0.01));
        walls.points.push_back(to_point(vertices[i], 0.01));
        walls.points.push_back(to_point(vertices[i + 1], 0.01));
      }
    }
    array.markers.push_back(walls);

    Marker goal = make_marker(stamp, "goal", 0, Marker::SPHERE);
    goal.pose.position = to_point(scenario_.goal, 0.05);
    goal.scale.x = goal.scale.y = goal.scale.z = 0.08;
    goal.color = rgba(0.75f, 0.18f, 0.16f, 0.95f);
    array.markers.push_back(goal);

    const std::size_t shown = std::min(trail_at_ + 1, mission_.trail.size());
    const std::size_t split = std::min(mission_.return_trail_index, shown);
    const std::size_t entry_end = std::min(split + 1, shown);
    Marker entry = make_marker(stamp, "entry_trail", 0, Marker::LINE_STRIP);
    entry.scale.x = 0.018;
    entry.color = rgba(0.22f, 0.42f, 0.78f, 0.9f);
    for (std::size_t i = 0; i < entry_end; ++i) {
      entry.points.push_back(to_point(mission_.trail[i], 0.03));
    }
    array.markers.push_back(entry);

    Marker returned = make_marker(stamp, "return_trail", 0, Marker::LINE_STRIP);
    returned.scale.x = 0.028;
    returned.color = rgba(0.12f, 0.55f, 0.32f, 0.95f);
    if (split < shown) {
      for (std::size_t i = split; i < shown; ++i) {
        returned.points.push_back(to_point(mission_.trail[i], 0.05));
      }
    }
    array.markers.push_back(returned);

    if (!mission_.frames.empty()) {
      const auto& frame = mission_.frames[frame_index_];
      Marker circle = make_marker(stamp, "vision_circle", 0, Marker::LINE_STRIP);
      circle.scale.x = 0.008;
      circle.color = rgba(0.55f, 0.22f, 0.22f, 0.55f);
      constexpr int kSteps = 48;
      for (int i = 0; i <= kSteps; ++i) {
        const double angle = 2.0 * std::acos(-1.0) * static_cast<double>(i) / kSteps;
        circle.points.push_back(to_point(
            {frame.pose.x + frame.vision_radius * std::cos(angle),
             frame.pose.y + frame.vision_radius * std::sin(angle)},
            0.04));
      }
      array.markers.push_back(circle);

      Marker closed = make_marker(stamp, "closed", 0, Marker::TRIANGLE_LIST);
      closed.color = rgba(0.75f, 0.4f, 0.48f, 0.16f);
      for (const auto& span : frame.closed) {
        add_wedge(closed, frame.pose, frame.vision_radius, span.start, span.sweep);
      }
      array.markers.push_back(closed);

      Marker open = make_marker(stamp, "open", 0, Marker::TRIANGLE_LIST);
      open.color = rgba(0.3f, 0.62f, 0.38f, 0.14f);
      for (const auto& span : frame.open) {
        add_wedge(open, frame.pose, frame.vision_radius, span.start, span.sweep);
      }
      array.markers.push_back(open);

      Marker points = make_marker(stamp, "open_points", 0, Marker::SPHERE_LIST);
      points.scale.x = points.scale.y = points.scale.z = 0.035;
      points.color = rgba(0.2f, 0.4f, 0.75f, 0.85f);
      for (const auto& point : frame.open_points) {
        points.points.push_back(to_point(point, 0.05));
      }
      array.markers.push_back(points);

      Marker bundles = make_marker(stamp, "bundles", 0, Marker::SPHERE_LIST);
      bundles.scale.x = bundles.scale.y = bundles.scale.z = 0.045;
      bundles.color = rgba(0.62f, 0.42f, 0.18f, 0.55f);
      for (const auto& center : frame.bundle_centers) {
        bundles.points.push_back(to_point(center, 0.06));
      }
      array.markers.push_back(bundles);

      Marker funnel = make_marker(stamp, "funnel", 0, Marker::LINE_STRIP);
      funnel.scale.x = 0.02;
      funnel.color = rgba(0.15f, 0.32f, 0.72f, 0.95f);
      for (const auto& point : frame.funnel) {
        funnel.points.push_back(to_point(point, 0.07));
      }
      array.markers.push_back(funnel);

      std_msgs::String status;
      status.data = "frame=" + std::to_string(frame.index) + " event=" + frame.event +
                    " stop=" + mission_.stop_reason;
      status_pub_.publish(status);
    }
    marker_pub_.publish(array);
  }

  void publish_pose_and_path(const ros::Time& stamp) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = stamp;
    nav_msgs::Path path;
    path.header = pose.header;
    if (mission_.frames.empty()) {
      pose.pose.position = to_point(scenario_.start, 0.0);
      pose.pose.orientation.w = 1.0;
    } else {
      const bool driving =
          hold_left_ == 0 && trail_at_ < mission_.trail.size();
      const auto& frame = mission_.frames[frame_index_];
      mars::common::Point2D position = frame.pose;
      double yaw = frame.yaw;
      if (driving && trail_at_ < mission_.trail.size()) {
        position = mission_.trail[trail_at_];
        if (trail_at_ + 1 < mission_.trail.size()) {
          const auto& ahead = mission_.trail[trail_at_ + 1];
          yaw = std::atan2(ahead.y - position.y, ahead.x - position.x);
        }
      }
      pose.pose.position = to_point(position, 0.0);
      pose.pose.orientation.z = std::sin(yaw * 0.5);
      pose.pose.orientation.w = std::cos(yaw * 0.5);
      for (const auto& point : frame.planned_path) {
        geometry_msgs::PoseStamped step;
        step.header = pose.header;
        step.pose.position = to_point(point, 0.0);
        step.pose.orientation.w = 1.0;
        path.poses.push_back(step);
      }
    }
    pose_pub_.publish(pose);
    path_pub_.publish(path);
  }

  ros::NodeHandle node_;
  mars::demo_simulation::Mode1Scenario scenario_{};
  mars::demo_simulation::LoadedMap map_{};
  mars::demo_simulation::MissionResult mission_{};
  std::size_t frame_index_{0};
  std::size_t trail_at_{0};
  int hold_left_{0};
  int hold_ticks_{20};
  double drive_speed_{0.12};
  static constexpr double kTickHz = 10.0;
  ros::Publisher map_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher path_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher status_pub_;
  ros::Timer timer_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "mode1_rviz");
  ros::NodeHandle node;
  Mode1RvizNode viewer(node);
  ros::spin();
  return 0;
}
