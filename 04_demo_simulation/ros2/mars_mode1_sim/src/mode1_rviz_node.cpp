#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "mode1_map.hpp"
#include "mode1_mission.hpp"

namespace {

using geometry_msgs::msg::Point;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

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

Marker make_marker(const rclcpp::Time& stamp, const std::string& ns, int id,
                   int type) {
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

class Mode1RvizNode : public rclcpp::Node {
 public:
  Mode1RvizNode() : Node("mode1_rviz") {
    declare_parameter<std::string>("map_id", "hard_alley");
    declare_parameter<std::string>("map_dir", "");
    declare_parameter<double>("playback_hz", 2.0);

    const auto latched =
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    const auto live = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", latched);
    marker_pub_ = create_publisher<MarkerArray>(
        "/dsfm_online/polygon_markers", live);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/planned_path", live);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        "/robot_current_pose", live);
    status_pub_ =
        create_publisher<std_msgs::msg::String>("/dsfm_online/status", latched);

    const std::string map_id = get_parameter("map_id").as_string();
    std::string map_dir = get_parameter("map_dir").as_string();
    if (map_dir.empty()) {
      map_dir = ament_index_cpp::get_package_share_directory("mars_mode1_sim") +
                "/maps";
    }
    scenario_ = mars::demo_simulation::mode1_scenario(map_id);
    map_ = mars::demo_simulation::load_occupancy_map(
        map_dir + "/" + scenario_.occupancy_yaml);
    RCLCPP_INFO(get_logger(), "Mode 1 map %s, obstacles=%zu", map_id.c_str(),
                map_.obstacles.size());
    mission_ = mars::demo_simulation::run_mode1_mission(scenario_, map_);
    RCLCPP_INFO(get_logger(), "stop=%s frames=%zu entry=%.2f return=%.2f",
                mission_.stop_reason.c_str(), mission_.frames.size(),
                mission_.entry_length, mission_.return_length);

    const double hz = std::max(0.2, get_parameter("playback_hz").as_double());
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / hz),
                               [this]() { publish_tick(); });
  }

 private:
  void publish_tick() {
    const rclcpp::Time stamp = now();
    publish_map(stamp);
    publish_markers(stamp);
    publish_pose_and_path(stamp);
    if (!mission_.frames.empty()) {
      cursor_ = (cursor_ + 1) % mission_.frames.size();
    }
  }

  void publish_map(const rclcpp::Time& stamp) {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.frame_id = "map";
    grid.header.stamp = stamp;
    grid.info.resolution = static_cast<float>(map_.resolution);
    grid.info.width = static_cast<uint32_t>(map_.width);
    grid.info.height = static_cast<uint32_t>(map_.height);
    grid.info.origin.position.x = map_.origin.x;
    grid.info.origin.position.y = map_.origin.y;
    grid.info.origin.orientation.w = 1.0;
    const auto cells =
        static_cast<std::size_t>(map_.width) * static_cast<std::size_t>(map_.height);
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
    map_pub_->publish(grid);
  }

  void publish_markers(const rclcpp::Time& stamp) {
    MarkerArray array;
    Marker clear = make_marker(stamp, "", 0, Marker::DELETEALL);
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    Marker walls = make_marker(stamp, "walls", 0, Marker::LINE_LIST);
    walls.scale.x = 0.02;
    walls.color = rgba(0.75f, 0.75f, 0.75f, 0.9f);
    for (const auto& obstacle : map_.obstacles) {
      const auto& vertices = obstacle.vertices;
      if (vertices.size() < 2) {
        continue;
      }
      const std::size_t edges =
          vertices.size() == 2 ? 1 : vertices.size();
      for (std::size_t i = 0; i < edges; ++i) {
        walls.points.push_back(to_point(vertices[i], 0.0));
        walls.points.push_back(
            to_point(vertices[(i + 1) % vertices.size()], 0.0));
      }
    }
    array.markers.push_back(walls);

    Marker goal = make_marker(stamp, "goal", 0, Marker::SPHERE);
    goal.pose.position = to_point(scenario_.goal, 0.05);
    goal.scale.x = goal.scale.y = goal.scale.z = 0.12;
    goal.color = rgba(1.0f, 0.1f, 0.1f, 0.95f);
    array.markers.push_back(goal);

    Marker trail = make_marker(stamp, "trail", 0, Marker::LINE_STRIP);
    trail.scale.x = 0.015;
    trail.color = rgba(1.0f, 0.1f, 0.1f, 0.55f);
    for (const auto& point : mission_.trail) {
      trail.points.push_back(to_point(point, 0.03));
    }
    array.markers.push_back(trail);

    if (!mission_.frames.empty()) {
      const auto& frame = mission_.frames[cursor_];
      Marker circle = make_marker(stamp, "vision_circle", 0, Marker::LINE_STRIP);
      circle.scale.x = 0.012;
      circle.color = rgba(1.0f, 0.15f, 0.15f, 0.8f);
      constexpr int kSteps = 48;
      for (int i = 0; i <= kSteps; ++i) {
        const double angle =
            2.0 * std::acos(-1.0) * static_cast<double>(i) / kSteps;
        circle.points.push_back(to_point(
            {frame.pose.x + frame.vision_radius * std::cos(angle),
             frame.pose.y + frame.vision_radius * std::sin(angle)},
            0.04));
      }
      array.markers.push_back(circle);

      Marker closed = make_marker(stamp, "closed", 0, Marker::TRIANGLE_LIST);
      closed.color = rgba(1.0f, 0.7f, 0.8f, 0.35f);
      for (const auto& span : frame.closed) {
        add_wedge(closed, frame.pose, frame.vision_radius, span.start,
                  span.sweep);
      }
      array.markers.push_back(closed);

      Marker open = make_marker(stamp, "open", 0, Marker::TRIANGLE_LIST);
      open.color = rgba(0.45f, 0.85f, 0.45f, 0.3f);
      for (const auto& span : frame.open) {
        add_wedge(open, frame.pose, frame.vision_radius, span.start, span.sweep);
      }
      array.markers.push_back(open);

      Marker points = make_marker(stamp, "open_points", 0, Marker::SPHERE_LIST);
      points.scale.x = points.scale.y = points.scale.z = 0.06;
      points.color = rgba(0.2f, 0.45f, 1.0f, 0.9f);
      for (const auto& point : frame.open_points) {
        points.points.push_back(to_point(point, 0.05));
      }
      array.markers.push_back(points);

      Marker graph = make_marker(stamp, "graph", 0, Marker::LINE_LIST);
      graph.scale.x = 0.008;
      graph.color = rgba(0.1f, 0.1f, 0.1f, 0.45f);
      for (const auto& edge : frame.graph) {
        graph.points.push_back(to_point(edge.first, 0.04));
        graph.points.push_back(to_point(edge.second, 0.04));
      }
      array.markers.push_back(graph);

      Marker bundles = make_marker(stamp, "bundles", 0, Marker::SPHERE_LIST);
      bundles.scale.x = bundles.scale.y = bundles.scale.z = 0.07;
      bundles.color = rgba(1.0f, 0.55f, 0.0f, 0.95f);
      Marker sequence = make_marker(stamp, "bundle_sequence", 0, Marker::LINE_STRIP);
      sequence.scale.x = 0.018;
      sequence.color = rgba(1.0f, 0.55f, 0.0f, 0.8f);
      for (const auto& center : frame.bundle_centers) {
        bundles.points.push_back(to_point(center, 0.06));
        sequence.points.push_back(to_point(center, 0.06));
      }
      array.markers.push_back(bundles);
      array.markers.push_back(sequence);

      Marker funnel = make_marker(stamp, "funnel", 0, Marker::LINE_STRIP);
      funnel.scale.x = 0.03;
      funnel.color = rgba(0.05f, 0.55f, 0.05f, 0.9f);
      for (const auto& point : frame.funnel) {
        funnel.points.push_back(to_point(point, 0.07));
      }
      array.markers.push_back(funnel);

      std_msgs::msg::String status;
      status.data = "frame=" + std::to_string(frame.index) +
                    " event=" + frame.event + " stop=" + mission_.stop_reason;
      status_pub_->publish(status);
    }
    marker_pub_->publish(array);
  }

  void publish_pose_and_path(const rclcpp::Time& stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = stamp;
    nav_msgs::msg::Path path;
    path.header = pose.header;
    if (mission_.frames.empty()) {
      pose.pose.position = to_point(scenario_.start, 0.0);
      pose.pose.orientation.w = 1.0;
    } else {
      const auto& frame = mission_.frames[cursor_];
      pose.pose.position = to_point(frame.pose, 0.0);
      pose.pose.orientation.z = std::sin(frame.yaw * 0.5);
      pose.pose.orientation.w = std::cos(frame.yaw * 0.5);
      for (const auto& point : frame.planned_path) {
        geometry_msgs::msg::PoseStamped step;
        step.header = pose.header;
        step.pose.position = to_point(point, 0.0);
        step.pose.orientation.w = 1.0;
        path.poses.push_back(step);
      }
    }
    pose_pub_->publish(pose);
    path_pub_->publish(path);
  }

  mars::demo_simulation::Mode1Scenario scenario_{};
  mars::demo_simulation::LoadedMap map_{};
  mars::demo_simulation::MissionResult mission_{};
  std::size_t cursor_{0};
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mode1RvizNode>());
  rclcpp::shutdown();
  return 0;
}
