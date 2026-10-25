#include "mode1_trace.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <ostream>
#include <sstream>
#include <stdexcept>

namespace mars::demo_simulation {
namespace {

constexpr double kPi = 3.14159265358979323846;

void write_point(std::ostream& out, mars::common::Point2D point) {
  out << "(" << point.x << ", " << point.y << ")";
}

int arc_samples(double sweep) {
  const int count = static_cast<int>(std::ceil(sweep / (kPi / 18.0)));
  if (count < 2) {
    return 2;
  }
  if (count > 72) {
    return 72;
  }
  return count;
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
    out << "marker bundles n=" << frame.bundles
        << " rays=" << frame.bundle_rays.size() << " centers=";
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
  bool any = false;
  const auto grow = [&](mars::common::Point2D point) {
    any = true;
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
  };
  const auto grow_segment = [&](const MarkerSegment& segment) {
    grow(segment.first);
    grow(segment.second);
  };
  for (const auto& point : mission.trail) {
    grow(point);
  }
  for (const auto& frame : mission.frames) {
    grow(frame.pose);
    if (frame.vision_radius > 0.0) {
      grow({frame.pose.x - frame.vision_radius, frame.pose.y - frame.vision_radius});
      grow({frame.pose.x + frame.vision_radius, frame.pose.y + frame.vision_radius});
    }
    for (const auto& point : frame.open_points) {
      grow(point);
    }
    for (const auto& segment : frame.graph) {
      grow_segment(segment);
    }
    for (const auto& segment : frame.bundle_rays) {
      grow_segment(segment);
    }
    for (const auto& point : frame.funnel) {
      grow(point);
    }
  }
  if (!any) {
    min_x = map.origin.x;
    min_y = map.origin.y;
    max_x = map.origin.x + static_cast<double>(map.width) * map.resolution;
    max_y = map.origin.y + static_cast<double>(map.height) * map.resolution;
  } else {
    constexpr double kMargin = 0.15;
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
  const auto world = [&](mars::common::Point2D center, double radius, double angle) {
    return mars::common::Point2D{center.x + radius * std::cos(angle),
                                 center.y + radius * std::sin(angle)};
  };

  std::ofstream out(path);
  if (!out) {
    throw std::runtime_error("cannot write marker svg " + path);
  }
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\">\n";
  out << "<rect width=\"100%\" height=\"100%\" fill=\"#f4f1ea\"/>\n";
  out << "<g id=\"walls\">\n";
  for (const auto& obstacle : map.obstacles) {
    bool near = false;
    for (const auto& vertex : obstacle.vertices) {
      if (vertex.x >= min_x && vertex.x <= max_x && vertex.y >= min_y &&
          vertex.y <= max_y) {
        near = true;
        break;
      }
    }
    if (!near || obstacle.vertices.empty()) {
      continue;
    }
    out << "<polygon fill=\"#2a3036\" points=\"";
    for (const auto& vertex : obstacle.vertices) {
      out << sx(vertex.x) << "," << sy(vertex.y) << " ";
    }
    out << "\"/>\n";
  }
  out << "</g>\n";

  auto write_wedge = [&](const MarkerFrame& frame, const AngleSpan& span,
                         const char* fill) {
    if (!(frame.vision_radius > 0.0) || !(span.sweep > 0.0)) {
      return;
    }
    const int samples = arc_samples(span.sweep);
    out << "<polygon fill=\"" << fill << "\" fill-opacity=\"0.35\" points=\""
        << sx(frame.pose.x) << "," << sy(frame.pose.y) << " ";
    for (int i = 0; i <= samples; ++i) {
      const double angle =
          span.start + span.sweep * (static_cast<double>(i) / static_cast<double>(samples));
      const auto point = world(frame.pose, frame.vision_radius, angle);
      out << sx(point.x) << "," << sy(point.y) << " ";
    }
    out << "\"/>\n";
  };

  out << "<g id=\"closed\">\n";
  for (const auto& frame : mission.frames) {
    for (const auto& span : frame.closed) {
      write_wedge(frame, span, "#f3b6c4");
    }
  }
  out << "</g>\n<g id=\"open\">\n";
  for (const auto& frame : mission.frames) {
    for (const auto& span : frame.open) {
      write_wedge(frame, span, "#8fd18f");
    }
  }
  out << "</g>\n<g id=\"vision-circle\" fill=\"none\" stroke=\"#d62728\" "
         "stroke-width=\"1.1\" stroke-dasharray=\"4 3\">\n";
  for (const auto& frame : mission.frames) {
    if (!(frame.vision_radius > 0.0)) {
      continue;
    }
    constexpr int kCircle = 48;
    out << "<polyline points=\"";
    for (int i = 0; i <= kCircle; ++i) {
      const double angle = kPi * 2.0 * (static_cast<double>(i) / static_cast<double>(kCircle));
      const auto point = world(frame.pose, frame.vision_radius, angle);
      out << sx(point.x) << "," << sy(point.y) << " ";
    }
    out << "\"/>\n";
  }
  out << "</g>\n";

  const MarkerFrame* last = mission.frames.empty() ? nullptr : &mission.frames.back();
  out << "<g id=\"graph\" fill=\"none\" stroke=\"#6b7280\" stroke-width=\"0.8\" "
         "stroke-dasharray=\"3 3\">\n";
  if (last != nullptr) {
    for (const auto& segment : last->graph) {
      out << "<line x1=\"" << sx(segment.first.x) << "\" y1=\"" << sy(segment.first.y)
          << "\" x2=\"" << sx(segment.second.x) << "\" y2=\"" << sy(segment.second.y)
          << "\"/>\n";
    }
  }
  out << "</g>\n<g id=\"bundles\" fill=\"none\" stroke=\"#1f77b4\" stroke-width=\"0.9\">\n";
  if (last != nullptr) {
    for (const auto& segment : last->bundle_rays) {
      out << "<line x1=\"" << sx(segment.first.x) << "\" y1=\"" << sy(segment.first.y)
          << "\" x2=\"" << sx(segment.second.x) << "\" y2=\"" << sy(segment.second.y)
          << "\"/>\n";
    }
  }
  out << "</g>\n<g id=\"funnel\" fill=\"none\" stroke=\"#1f4fbf\" stroke-width=\"1.8\" "
         "stroke-dasharray=\"7 5\">\n";
  if (last != nullptr && last->funnel.size() >= 2) {
    out << "<polyline points=\"";
    for (const auto& point : last->funnel) {
      out << sx(point.x) << "," << sy(point.y) << " ";
    }
    out << "\"/>\n";
  }
  out << "</g>\n<g id=\"trail\">\n";
  if (mission.trail.size() >= 2) {
    const std::size_t split =
        std::min(mission.return_trail_index, mission.trail.size());
    auto write_trail = [&](std::size_t begin, std::size_t end, const char* color,
                           const char* stroke) {
      if (end <= begin) {
        return;
      }
      out << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\""
          << stroke << "\" stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
      for (std::size_t i = begin; i < end; ++i) {
        out << sx(mission.trail[i].x) << "," << sy(mission.trail[i].y) << " ";
      }
      out << "\"/>\n";
    };
    write_trail(0, split, "#2f62b5", "3.2");
    const std::size_t return_begin =
        (split > 0 && split < mission.trail.size()) ? split - 1 : split;
    write_trail(return_begin, mission.trail.size(), "#1a8f4e", "3.6");
  }
  if (!mission.trail.empty()) {
    out << "<circle cx=\"" << sx(mission.trail.front().x) << "\" cy=\""
        << sy(mission.trail.front().y) << "\" r=\"5\" fill=\"#2f62b5\"/>\n";
  }
  out << "</g>\n<g id=\"open-points\">\n";
  for (const auto& frame : mission.frames) {
    for (const auto& point : frame.open_points) {
      out << "<circle cx=\"" << sx(point.x) << "\" cy=\"" << sy(point.y)
          << "\" r=\"2.4\" fill=\"#1d4e89\"/>\n";
    }
  }
  out << "</g>\n";
  out << "<text x=\"12\" y=\"18\" font-family=\"sans-serif\" font-size=\"12\" "
         "fill=\"#222\">modules 1-3: pink closed, green open, blue open points, "
         "gray graph, blue bundle rays, green return</text>\n";
  out << "</svg>\n";
}

void write_algorithm_panels(const std::string& directory, const LoadedMap& map,
                            const MissionResult& mission,
                            mars::common::Point2D goal) {
  std::vector<const MarkerFrame*> shown;
  for (const auto& frame : mission.frames) {
    if (frame.event == "RETURN_REACHED") {
      continue;
    }
    shown.push_back(&frame);
    if (frame.event == "FUNNEL") {
      break;
    }
  }
  const MarkerFrame* focus = shown.empty() ? nullptr : shown.back();

  const double min_x = 0.0;
  const double min_y = 0.0;
  const double max_x = 100.0;
  const double max_y = 100.0;
  const double scale = 7.2;
  const double pad = 28.0;
  const double width = (max_x - min_x) * scale + 2.0 * pad;
  const double height = (max_y - min_y) * scale + 2.0 * pad + 22.0;
  const auto sx = [&](double x) { return pad + (x - min_x) * scale; };
  const auto sy = [&](double y) { return pad + 18.0 + (max_y - y) * scale; };
  const auto world = [&](mars::common::Point2D center, double radius, double angle) {
    return mars::common::Point2D{center.x + radius * std::cos(angle),
                                 center.y + radius * std::sin(angle)};
  };

  std::size_t entry_end = mission.trail.size();
  if (!shown.empty()) {
    entry_end = std::min(shown.back()->trail_index + 1, mission.trail.size());
  }
  if (mission.return_trail_index < entry_end) {
    entry_end = mission.return_trail_index;
  }
  std::size_t return_end = mission.trail.size();
  for (std::size_t i = 0; i < mission.frames.size(); ++i) {
    if (mission.frames[i].event == "FUNNEL") {
      if (i + 1 < mission.frames.size()) {
        return_end = std::min(mission.frames[i + 1].trail_index + 1, mission.trail.size());
      }
      break;
    }
  }
  auto slash = directory;
  if (!slash.empty() && slash.back() != '/' && slash.back() != '\\') {
    slash += '/';
  }

  struct Panel {
    const char* file;
    const char* title;
    bool sights;
    bool points;
    bool graph;
    bool bundles;
    bool exit_path;
  };
  const Panel panels[] = {
      {"fig8a_sights.svg", "Hinh 8a — tam nhin, closed/open sight, open point",
       true, true, false, false, false},
      {"fig8b_graph.svg", "Hinh 8b — do thi tam nhin toi open point", false, false,
       true, false, false},
      {"fig9b_bundles.svg", "Hinh 9b — chum tia bundle", false, false, false, true,
       false},
      {"fig9c_exit.svg", "Hinh 9c — duong di ra ngan hon duong vao", false, false,
       false, true, true},
  };

  for (const Panel& panel : panels) {
    std::ofstream out(slash + panel.file);
    if (!out) {
      throw std::runtime_error("cannot write " + slash + panel.file);
    }
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
        << "\" height=\"" << height << "\">\n";
    out << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
    out << "<text x=\"" << pad << "\" y=\"20\" font-family=\"sans-serif\" "
           "font-size=\"15\" fill=\"#222\">"
        << panel.title << "</text>\n";
    out << "<g id=\"walls\">\n";
    for (const auto& obstacle : map.obstacles) {
      if (obstacle.vertices.empty()) {
        continue;
      }
      out << "<polygon fill=\"#d9d9d9\" stroke=\"#4a4a4a\" stroke-width=\"0.8\" "
             "points=\"";
      for (const auto& vertex : obstacle.vertices) {
        out << sx(vertex.x) << "," << sy(vertex.y) << " ";
      }
      out << "\"/>\n";
    }
    out << "</g>\n";

    if (panel.sights) {
      out << "<g id=\"closed\">\n";
      for (const auto* frame : shown) {
        for (const auto& span : frame->closed) {
          if (!(frame->vision_radius > 0.0) || !(span.sweep > 0.0)) {
            continue;
          }
          const int samples = arc_samples(span.sweep);
          out << "<polygon fill=\"#f3b6c4\" fill-opacity=\"0.55\" points=\""
              << sx(frame->pose.x) << "," << sy(frame->pose.y) << " ";
          for (int i = 0; i <= samples; ++i) {
            const double angle =
                span.start + span.sweep * (static_cast<double>(i) /
                                            static_cast<double>(samples));
            const auto point = world(frame->pose, frame->vision_radius, angle);
            out << sx(point.x) << "," << sy(point.y) << " ";
          }
          out << "\"/>\n";
        }
      }
      out << "</g>\n<g id=\"open\">\n";
      for (const auto* frame : shown) {
        for (const auto& span : frame->open) {
          if (!(frame->vision_radius > 0.0) || !(span.sweep > 0.0)) {
            continue;
          }
          const int samples = arc_samples(span.sweep);
          out << "<polygon fill=\"#8fd18f\" fill-opacity=\"0.40\" points=\""
              << sx(frame->pose.x) << "," << sy(frame->pose.y) << " ";
          for (int i = 0; i <= samples; ++i) {
            const double angle =
                span.start + span.sweep * (static_cast<double>(i) /
                                            static_cast<double>(samples));
            const auto point = world(frame->pose, frame->vision_radius, angle);
            out << sx(point.x) << "," << sy(point.y) << " ";
          }
          out << "\"/>\n";
        }
      }
      out << "</g>\n";
    }

    out << "<g id=\"vision-circle\" fill=\"none\" stroke=\"#d62728\" "
           "stroke-width=\"1.1\" stroke-dasharray=\"4 3\">\n";
    for (const auto* frame : shown) {
      if (!(frame->vision_radius > 0.0)) {
        continue;
      }
      constexpr int kCircle = 48;
      out << "<polyline points=\"";
      for (int i = 0; i <= kCircle; ++i) {
        const double angle =
            kPi * 2.0 * (static_cast<double>(i) / static_cast<double>(kCircle));
        const auto point = world(frame->pose, frame->vision_radius, angle);
        out << sx(point.x) << "," << sy(point.y) << " ";
      }
      out << "\"/>\n";
    }
    out << "</g>\n";

    auto write_trail = [&](std::size_t begin, std::size_t end, const char* color,
                           const char* dash, const char* stroke) {
      if (end <= begin) {
        return;
      }
      out << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\""
          << stroke << "\"";
      if (dash != nullptr) {
        out << " stroke-dasharray=\"" << dash << "\"";
      }
      out << " stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
      bool drew = false;
      mars::common::Point2D previous{};
      for (std::size_t i = begin; i < end; ++i) {
        const bool last = i + 1 == end;
        if (drew && !last &&
            std::hypot(mission.trail[i].x - previous.x,
                       mission.trail[i].y - previous.y) < 0.8) {
          continue;
        }
        out << sx(mission.trail[i].x) << "," << sy(mission.trail[i].y) << " ";
        previous = mission.trail[i];
        drew = true;
      }
      out << "\"/>\n";
    };

    if (!panel.exit_path) {
      write_trail(0, entry_end, "#d62728", nullptr, "2.6");
    }

    if (panel.graph && focus != nullptr) {
      out << "<g id=\"graph\" fill=\"none\" stroke=\"#333333\" stroke-width=\"0.9\" "
             "stroke-dasharray=\"3 2\">\n";
      for (const auto& segment : focus->graph) {
        out << "<line x1=\"" << sx(segment.first.x) << "\" y1=\"" << sy(segment.first.y)
            << "\" x2=\"" << sx(segment.second.x) << "\" y2=\"" << sy(segment.second.y)
            << "\"/>\n";
      }
      out << "</g>\n";
    }
    if (panel.bundles && focus != nullptr) {
      out << "<g id=\"bundles\" fill=\"none\" stroke=\"#1f77b4\" stroke-width=\"1.0\">\n";
      for (const auto& segment : focus->bundle_rays) {
        out << "<line x1=\"" << sx(segment.first.x) << "\" y1=\"" << sy(segment.first.y)
            << "\" x2=\"" << sx(segment.second.x) << "\" y2=\"" << sy(segment.second.y)
            << "\"/>\n";
      }
      out << "</g>\n";
    }

    if (panel.exit_path) {
      write_trail(0, entry_end, "#ffffff", nullptr, "7");
      write_trail(0, entry_end, "#222222", "9 6", "2.8");
      const std::size_t return_begin = entry_end > 0 ? entry_end - 1 : entry_end;
      double alley_entry_len = 0.0;
      for (std::size_t i = 1; i < entry_end; ++i) {
        alley_entry_len += std::hypot(mission.trail[i].x - mission.trail[i - 1].x,
                                      mission.trail[i].y - mission.trail[i - 1].y);
      }
      double alley_exit_len = 0.0;
      if (return_end > return_begin) {
        for (std::size_t i = return_begin + 1; i < return_end; ++i) {
          alley_exit_len += std::hypot(mission.trail[i].x - mission.trail[i - 1].x,
                                       mission.trail[i].y - mission.trail[i - 1].y);
        }
      }
      if (alley_exit_len > 0.0 && return_end > return_begin) {
        write_trail(return_begin, return_end, "#ffffff", nullptr, "11");
        write_trail(return_begin, return_end, "#e10600", nullptr, "6");
      } else if (focus != nullptr && focus->funnel.size() >= 2) {
        auto write_funnel = [&](const char* color, const char* stroke) {
          out << "<polyline fill=\"none\" stroke=\"" << color << "\" stroke-width=\""
              << stroke << "\" stroke-linejoin=\"round\" stroke-linecap=\"round\" points=\"";
          for (const auto& point : focus->funnel) {
            out << sx(point.x) << "," << sy(point.y) << " ";
          }
          out << "\"/>\n";
        };
        write_funnel("#ffffff", "11");
        write_funnel("#e10600", "6");
      }
      out << "<text x=\"" << sx(8) << "\" y=\"" << sy(12)
          << "\" font-family=\"sans-serif\" font-size=\"14\" fill=\"#111\">"
          << "L vao " << alley_entry_len;
      if (alley_exit_len > 0.0) {
        out << "   L ra " << alley_exit_len;
      }
      out << "</text>\n";
    }

    if (panel.points) {
      out << "<g id=\"open-points\">\n";
      for (const auto* frame : shown) {
        for (const auto& point : frame->open_points) {
          out << "<circle cx=\"" << sx(point.x) << "\" cy=\"" << sy(point.y)
              << "\" r=\"3.2\" fill=\"#1d4e89\"/>\n";
        }
      }
      out << "</g>\n";
    }

    int pose_index = 0;
    for (const auto* frame : shown) {
      if (frame->event == "FUNNEL") {
        continue;
      }
      out << "<circle cx=\"" << sx(frame->pose.x) << "\" cy=\"" << sy(frame->pose.y)
          << "\" r=\"4.5\" fill=\"#1d4e89\"/>\n";
      out << "<text x=\"" << (sx(frame->pose.x) + 6) << "\" y=\"" << (sy(frame->pose.y) - 6)
          << "\" font-family=\"sans-serif\" font-size=\"12\" fill=\"#1d4e89\">C"
          << pose_index << "</text>\n";
      ++pose_index;
    }
    out << "<polygon fill=\"#d62728\" points=\"";
    const double star = 1.6;
    for (int i = 0; i < 10; ++i) {
      const double radius = (i % 2 == 0) ? star : star * 0.4;
      const double angle = -kPi / 2.0 + i * kPi / 5.0;
      const auto point = world(goal, radius, angle);
      out << sx(point.x) << "," << sy(point.y) << " ";
    }
    out << "\"/>\n</svg>\n";
  }
}

}  // namespace mars::demo_simulation
