#include "mode1_trace.hpp"

#include <algorithm>
#include <fstream>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace mars::demo_simulation {
namespace {

void write_point(std::ostream& out, mars::common::Point2D point) {
  out << "(" << point.x << ", " << point.y << ")";
}

}  // namespace

std::string format_marker_log(const MissionResult& mission) {
  std::ostringstream out;
  out << "stop=" << mission.stop_reason
      << " follower_stuck=" << (mission.follower_stuck ? 1 : 0)
      << " entry=" << mission.entry_length
      << " return=" << mission.return_length
      << " saw_entry=" << (mission.saw_entry ? 1 : 0)
      << " saw_return=" << (mission.saw_return ? 1 : 0) << "\n";
  for (const auto& frame : mission.frames) {
    out << "frame " << frame.index << " event=" << frame.event << " pose=";
    write_point(out, frame.pose);
    out << "\nmarker vision_circle center=";
    write_point(out, frame.pose);
    out << " r=" << frame.vision_radius << "\n";
    out << "marker closed n=" << frame.closed.size();
    for (const auto& span : frame.closed) {
      out << " [" << span.start << " +" << span.sweep << "]";
    }
    out << "\nmarker open n=" << frame.open.size();
    for (const auto& span : frame.open) {
      out << " [" << span.start << " +" << span.sweep << "]";
    }
    out << "\nmarker open_points n=" << frame.open_points.size();
    for (const auto& point : frame.open_points) {
      out << " ";
      write_point(out, point);
    }
    out << "\nmarker graph nodes=" << frame.graph_nodes
        << " edges=" << frame.graph_edges << "\n";
    out << "marker bundles n=" << frame.bundles << " centers=";
    for (const auto& center : frame.bundle_centers) {
      out << " ";
      write_point(out, center);
    }
    out << "\nmarker funnel n=" << frame.funnel.size();
    for (const auto& point : frame.funnel) {
      out << " ";
      write_point(out, point);
    }
    out << "\n";
  }
  return out.str();
}

void write_marker_svg(const std::string& path, const LoadedMap& map,
                      const MissionResult& mission) {
  double min_x = 1.0e300;
  double min_y = 1.0e300;
  double max_x = -1.0e300;
  double max_y = -1.0e300;
  const auto grow = [&](mars::common::Point2D point) {
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  };
  for (const auto& point : mission.trail) {
    grow(point);
  }
  if (mission.trail.empty()) {
    min_x = map.origin.x;
    min_y = map.origin.y;
    max_x = map.origin.x + static_cast<double>(map.width) * map.resolution;
    max_y = map.origin.y + static_cast<double>(map.height) * map.resolution;
  } else {
    constexpr double kMargin = 0.55;
    min_x -= kMargin;
    min_y -= kMargin;
    max_x += kMargin;
    max_y += kMargin;
  }
  const double scale = 140.0;
  const double pad = 16.0;
  const double width = (max_x - min_x) * scale + 2.0 * pad;
  const double height = (max_y - min_y) * scale + 2.0 * pad;
  const auto sx = [&](double x) { return pad + (x - min_x) * scale; };
  const auto sy = [&](double y) { return pad + (max_y - y) * scale; };

  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot write marker svg " + path);
  }
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\">\n";
  out << "<rect width=\"100%\" height=\"100%\" fill=\"#f4f1ea\"/>\n";
  for (const auto& obstacle : map.obstacles) {
    bool near = false;
    for (const auto& vertex : obstacle.vertices) {
      if (vertex.x >= min_x && vertex.x <= max_x && vertex.y >= min_y && vertex.y <= max_y) {
        near = true;
        break;
      }
    }
    if (!near) {
      continue;
    }
    out << "<polygon fill=\"#2a3036\" points=\"";
    for (const auto& vertex : obstacle.vertices) {
      out << sx(vertex.x) << "," << sy(vertex.y) << " ";
    }
    out << "\"/>\n";
  }
  if (!mission.frames.empty()) {
    const auto& frame = mission.frames.back();
    if (frame.funnel.size() >= 2) {
      out << "<polyline fill=\"none\" stroke=\"#1f4fbf\" stroke-width=\"1.6\" "
             "stroke-dasharray=\"7 5\" points=\"";
      for (const auto& point : frame.funnel) {
        out << sx(point.x) << "," << sy(point.y) << " ";
      }
      out << "\"/>\n";
    }
  }
  if (mission.trail.size() >= 2) {
    const std::size_t split =
        std::min(mission.return_trail_index, mission.trail.size());
    auto write_trail = [&](std::size_t begin, std::size_t end, const char* color,
                           const char* width) {
      if (end <= begin) {
        return;
      }
      out << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\""
          << width << "\" stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
      for (std::size_t i = begin; i < end; ++i) {
        out << sx(mission.trail[i].x) << "," << sy(mission.trail[i].y) << " ";
      }
      out << "\"/>\n";
    };
    write_trail(0, split, "#2f62b5", "3.2");
    write_trail(split, mission.trail.size(), "#1a8f4e", "3.6");
    if (!mission.trail.empty()) {
      out << "<circle cx=\"" << sx(mission.trail.front().x) << "\" cy=\""
          << sy(mission.trail.front().y) << "\" r=\"5\" fill=\"#2f62b5\"/>\n";
    }
  }
  out << "</svg>\n";
}

}  // namespace mars::demo_simulation
