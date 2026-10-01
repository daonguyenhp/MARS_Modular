#include "mars/graph_bundle_management/gate_preparation.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "internal/geometry_helpers.hpp"

namespace mars::graph_bundle_management {
namespace {

using mars::common::Point2D;

constexpr double kJoin = 1.0e-7;
constexpr double kWeld = 1.0e-5;

struct Triangle {
  Point2D a{};
  Point2D b{};
  Point2D c{};
};

double vcross(Point2D u, Point2D v) noexcept {
  return u.x * v.y - u.y * v.x;
}

double signed_area(const std::vector<Point2D>& polygon) {
  double area = 0.0;
  for (std::size_t index = 0; index < polygon.size(); ++index) {
    const auto& here = polygon[index];
    const auto& next = polygon[(index + 1) % polygon.size()];
    area += here.x * next.y - next.x * here.y;
  }
  return 0.5 * area;
}

bool in_triangle(Point2D point, const Triangle& triangle, double epsilon) {
  const double ab = internal::cross(triangle.a, triangle.b, point);
  const double bc = internal::cross(triangle.b, triangle.c, point);
  const double ca = internal::cross(triangle.c, triangle.a, point);
  const bool non_negative = ab >= -epsilon && bc >= -epsilon && ca >= -epsilon;
  const bool non_positive = ab <= epsilon && bc <= epsilon && ca <= epsilon;
  return non_negative || non_positive;
}

bool proper_hit(Point2D a, Point2D b, Point2D c, Point2D d, Point2D& at,
                double& t_ab, double& t_cd);

Point2D centroid(const Triangle& triangle) {
  return {(triangle.a.x + triangle.b.x + triangle.c.x) / 3.0,
          (triangle.a.y + triangle.b.y + triangle.c.y) / 3.0};
}

std::vector<Point2D> around_center(const Bundle& bundle) {
  std::vector<Point2D> vertices = bundle.ordered_vertices;
  const Point2D center = bundle.concurrent_point;
  std::sort(vertices.begin(), vertices.end(), [&](Point2D first, Point2D second) {
    const double first_angle = std::atan2(first.y - center.y, first.x - center.x);
    const double second_angle = std::atan2(second.y - center.y, second.x - center.x);
    if (first_angle != second_angle) {
      return first_angle < second_angle;
    }
    return internal::squared_distance(first, center) <
           internal::squared_distance(second, center);
  });
  vertices.erase(std::unique(vertices.begin(), vertices.end(),
                             [&](Point2D first, Point2D second) {
                               return internal::point_near(first, second, kJoin);
                             }),
                 vertices.end());
  return vertices;
}

bool crosses_obstacle(Point2D a, Point2D b, const std::vector<Bundle>& bundles) {
  for (const auto& bundle : bundles) {
    for (const auto& edge : bundle.obstacle_edges) {
      Point2D at{};
      double t_ab = 0.0;
      double t_cd = 0.0;
      if (proper_hit(a, b, edge.start, edge.end, at, t_ab, t_cd)) {
        return true;
      }
    }
  }
  return false;
}

double distance_to_segment(Point2D point, Point2D a, Point2D b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length2 = dx * dx + dy * dy;
  if (length2 <= 1.0e-18) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }
  const double t = std::max(
      0.0, std::min(1.0, ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2));
  return std::hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
}

bool proper_hit(Point2D a, Point2D b, Point2D c, Point2D d, Point2D& at,
                double& t_ab, double& t_cd) {
  if (internal::point_near(a, c, kJoin) || internal::point_near(a, d, kJoin) ||
      internal::point_near(b, c, kJoin) || internal::point_near(b, d, kJoin)) {
    return false;
  }
  const double ab_c = internal::cross(a, b, c);
  const double ab_d = internal::cross(a, b, d);
  const double cd_a = internal::cross(c, d, a);
  const double cd_b = internal::cross(c, d, b);
  if (!(ab_c * ab_d < -1.0e-12 && cd_a * cd_b < -1.0e-12)) {
    return false;
  }
  const double denominator =
      (a.x - b.x) * (c.y - d.y) - (a.y - b.y) * (c.x - d.x);
  if (std::abs(denominator) <= 1.0e-15) {
    return false;
  }
  t_ab = ((a.x - c.x) * (c.y - d.y) - (a.y - c.y) * (c.x - d.x)) / denominator;
  t_cd = ((a.x - c.x) * (a.y - b.y) - (a.y - c.y) * (a.x - b.x)) / denominator;
  at = {a.x + t_ab * (b.x - a.x), a.y + t_ab * (b.y - a.y)};
  return t_ab > 1.0e-8 && t_ab < 1.0 - 1.0e-8 && t_cd > 1.0e-8 &&
         t_cd < 1.0 - 1.0e-8;
}

bool strictly_inside(Point2D point, Point2D a, Point2D b, Point2D c) {
  const double ab = internal::cross(a, b, point);
  const double bc = internal::cross(b, c, point);
  const double ca = internal::cross(c, a, point);
  return ab > 1.0e-10 && bc > 1.0e-10 && ca > 1.0e-10;
}

std::optional<std::vector<Triangle>> triangulate(const std::vector<Point2D>& polygon) {
  const std::size_t count = polygon.size();
  if (count < 3 || std::abs(signed_area(polygon)) <= 1.0e-10) {
    return std::nullopt;
  }
  if (count == 3) {
    return std::vector<Triangle>{{polygon[0], polygon[1], polygon[2]}};
  }
  std::vector<std::size_t> ring(count);
  for (std::size_t index = 0; index < count; ++index) {
    ring[index] = index;
  }
  std::vector<Triangle> triangles;
  while (ring.size() > 3) {
    bool clipped = false;
    for (std::size_t index = 0; index < ring.size(); ++index) {
      const std::size_t prev = ring[(index + ring.size() - 1) % ring.size()];
      const std::size_t curr = ring[index];
      const std::size_t next = ring[(index + 1) % ring.size()];
      if (internal::cross(polygon[prev], polygon[curr], polygon[next]) <= 1.0e-10) {
        continue;
      }
      bool blocked = false;
      for (const std::size_t other : ring) {
        if (other == prev || other == curr || other == next) {
          continue;
        }
        if (strictly_inside(polygon[other], polygon[prev], polygon[curr],
                            polygon[next])) {
          blocked = true;
          break;
        }
      }
      if (blocked) {
        continue;
      }
      triangles.push_back({polygon[prev], polygon[curr], polygon[next]});
      ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(index));
      clipped = true;
      break;
    }
    if (!clipped) {
      return std::nullopt;
    }
  }
  triangles.push_back({polygon[ring[0]], polygon[ring[1]], polygon[ring[2]]});
  return triangles;
}

bool has_vertex(const Triangle& triangle, Point2D point) {
  return internal::point_near(triangle.a, point, kJoin) ||
         internal::point_near(triangle.b, point, kJoin) ||
         internal::point_near(triangle.c, point, kJoin);
}

std::optional<std::pair<Point2D, Point2D>> shared_edge(const Triangle& first,
                                                       const Triangle& second) {
  const Point2D vertices[] = {first.a, first.b, first.c};
  std::vector<Point2D> shared;
  for (const auto& vertex : vertices) {
    if (has_vertex(second, vertex)) {
      shared.push_back(vertex);
    }
  }
  if (shared.size() < 2) {
    return std::nullopt;
  }
  return std::make_pair(shared[0], shared[1]);
}

bool is_center(Point2D point, const std::vector<Bundle>& bundles) {
  return std::any_of(bundles.begin(), bundles.end(), [&](const Bundle& bundle) {
    return internal::point_near(point, bundle.concurrent_point, kJoin);
  });
}

ObservationId nearest_bundle(Point2D point, const std::vector<Bundle>& bundles) {
  std::size_t best = 0;
  double best_distance =
      internal::distance(point, bundles.front().concurrent_point);
  for (std::size_t index = 1; index < bundles.size(); ++index) {
    const double length =
        internal::distance(point, bundles[index].concurrent_point);
    if (length < best_distance) {
      best_distance = length;
      best = index;
    }
  }
  return bundles[best].observation_id;
}

std::optional<std::vector<Gate>> sleeve_portals(const std::vector<Triangle>& triangles,
                                                Point2D start, Point2D goal,
                                                const std::vector<Bundle>& bundles) {
  const std::size_t count = triangles.size();
  std::vector<std::vector<std::size_t>> neighbors(count);
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      if (shared_edge(triangles[i], triangles[j])) {
        neighbors[i].push_back(j);
        neighbors[j].push_back(i);
      }
    }
  }

  constexpr std::size_t kNone = static_cast<std::size_t>(-1);
  constexpr std::size_t kStart = static_cast<std::size_t>(-2);
  std::vector<std::size_t> parent(count, kNone);
  std::vector<std::size_t> goals;
  for (std::size_t index = 0; index < count; ++index) {
    if (has_vertex(triangles[index], start) ||
        in_triangle(start, triangles[index], kJoin)) {
      parent[index] = kStart;
    }
    if (has_vertex(triangles[index], goal) ||
        in_triangle(goal, triangles[index], kJoin)) {
      goals.push_back(index);
    }
  }

  const auto gates_along = [&](std::size_t reached_triangle,
                              const std::vector<std::size_t>& parents) {
    std::vector<std::size_t> sleeve;
    for (std::size_t cursor = reached_triangle; cursor != kStart;) {
      sleeve.push_back(cursor);
      cursor = parents[cursor];
    }
    std::reverse(sleeve.begin(), sleeve.end());
    std::vector<Gate> gates;
    for (std::size_t index = 1; index < sleeve.size(); ++index) {
      const auto edge = shared_edge(triangles[sleeve[index - 1]], triangles[sleeve[index]]);
      const auto intermediate_center = [&](Point2D point) {
        return !internal::point_near(point, start, kJoin) &&
               !internal::point_near(point, goal, kJoin) && is_center(point, bundles);
      };
      if (!edge || intermediate_center(edge->first) || intermediate_center(edge->second) ||
          (is_center(edge->first, bundles) && is_center(edge->second, bundles))) {
        continue;
      }
      const Point2D from = centroid(triangles[sleeve[index - 1]]);
      const Point2D to = centroid(triangles[sleeve[index]]);
      const Point2D travel{to.x - from.x, to.y - from.y};
      const Point2D mid{(edge->first.x + edge->second.x) * 0.5,
                        (edge->first.y + edge->second.y) * 0.5};
      const double side =
          vcross(travel, {edge->first.x - mid.x, edge->first.y - mid.y});
      Gate gate;
      gate.left = side >= 0.0 ? edge->first : edge->second;
      gate.right = side >= 0.0 ? edge->second : edge->first;
      gate.source_observation_id = nearest_bundle(mid, bundles);
      gate.sequence_index = gates.size();
      gate.orientation = GateOrientation::LeftToRight;
      gate.valid = internal::distance(gate.left, gate.right) > kJoin;
      if (gate.valid) {
        gates.push_back(gate);
      }
    }
    return gates;
  };

  const auto portal_length = [&](const std::vector<Gate>& gates) {
    double length = 0.0;
    Point2D cursor = start;
    for (const auto& gate : gates) {
      const Point2D mid{(gate.left.x + gate.right.x) * 0.5, (gate.left.y + gate.right.y) * 0.5};
      length += internal::distance(cursor, mid);
      cursor = mid;
    }
    return length + internal::distance(cursor, goal);
  };

  std::vector<std::size_t> starts;
  for (std::size_t index = 0; index < count; ++index) {
    if (parent[index] == kStart) {
      starts.push_back(index);
    }
  }

  std::optional<std::vector<Gate>> best;
  double best_length = 0.0;
  bool connected = false;
  for (const std::size_t source : starts) {
    std::vector<std::size_t> parents(count, kNone);
    std::queue<std::size_t> walk;
    parents[source] = kStart;
    walk.push(source);
    std::size_t reached = kNone;
    while (!walk.empty() && reached == kNone) {
      const std::size_t current = walk.front();
      walk.pop();
      if (std::find(goals.begin(), goals.end(), current) != goals.end()) {
        reached = current;
        break;
      }
      for (const std::size_t next : neighbors[current]) {
        if (parents[next] != kNone || parent[next] == kStart) {
          continue;
        }
        parents[next] = current;
        walk.push(next);
      }
    }
    if (reached == kNone) {
      continue;
    }
    connected = true;
    const auto gates = gates_along(reached, parents);
    if (gates.empty()) {
      continue;
    }
    const double length = portal_length(gates);
    if (!best || length + 1.0e-9 < best_length) {
      best_length = length;
      best = gates;
    }
  }
  if (!best && connected) {
    return std::vector<Gate>{};
  }
  return best;
}

double signed_turn(Point2D from, Point2D to) {
  return std::atan2(vcross(from, to), from.x * to.x + from.y * to.y);
}

Point2D unit_of(Point2D vector) {
  const double length = std::hypot(vector.x, vector.y);
  if (length <= 1.0e-12) {
    return {1.0, 0.0};
  }
  return {vector.x / length, vector.y / length};
}

bool triangle_blocked(Point2D a, Point2D b, Point2D c,
                      const std::vector<Bundle>& bundles) {
  if (crosses_obstacle(a, b, bundles) || crosses_obstacle(b, c, bundles) ||
      crosses_obstacle(c, a, bundles)) {
    return true;
  }
  for (const auto& bundle : bundles) {
    for (const auto& edge : bundle.obstacle_edges) {
      if (strictly_inside(edge.start, a, b, c) ||
          strictly_inside(edge.end, a, b, c)) {
        return true;
      }
    }
  }
  return false;
}

bool on_leq_pi_side(Point2D vertex, Point2D prev, Point2D center, Point2D next) {
  const Point2D back{prev.x - center.x, prev.y - center.y};
  const Point2D forward{next.x - center.x, next.y - center.y};
  const Point2D spoke{vertex.x - center.x, vertex.y - center.y};
  if (std::hypot(spoke.x, spoke.y) <= 1.0e-12 ||
      std::hypot(back.x, back.y) <= 1.0e-12 ||
      std::hypot(forward.x, forward.y) <= 1.0e-12) {
    return false;
  }
  const double wedge = signed_turn(back, forward);
  if (std::abs(wedge) <= 1.0e-6) {
    return false;
  }
  // A straight skeleton has angle pi on both sides, so both forward walls
  // belong to that side. Vertices behind the robot stay out.
  if (std::abs(std::abs(wedge) - std::acos(-1.0)) <= 0.12) {
    return spoke.x * forward.x + spoke.y * forward.y >= -1.0e-8;
  }
  const double at = signed_turn(back, spoke);
  const double lower = std::min(0.0, wedge) - 1.0e-6;
  const double upper = std::max(0.0, wedge) + 1.0e-6;
  return at >= lower && at <= upper;
}

double free_reach(Point2D center, Point2D direction, double reach,
                  const std::vector<Bundle>& bundles) {
  const Point2D far{center.x + direction.x * reach, center.y + direction.y * reach};
  double best = 1.0;
  for (const auto& bundle : bundles) {
    for (const auto& edge : bundle.obstacle_edges) {
      Point2D at{};
      double t_ab = 0.0;
      double t_cd = 0.0;
      if (proper_hit(center, far, edge.start, edge.end, at, t_ab, t_cd)) {
        best = std::min(best, t_ab);
      }
    }
  }
  return best * reach;
}

Point2D rotate_by(Point2D vector, double angle) {
  const double cosine = std::cos(angle);
  const double sine = std::sin(angle);
  return {vector.x * cosine - vector.y * sine, vector.x * sine + vector.y * cosine};
}

Point2D clip_sight(Point2D center, Point2D toward, const std::vector<Bundle>& bundles) {
  const Point2D spoke{toward.x - center.x, toward.y - center.y};
  const double reach = std::hypot(spoke.x, spoke.y);
  if (reach <= 1.0e-12) {
    return center;
  }
  const Point2D direction = unit_of(spoke);
  const double clear = free_reach(center, direction, reach, bundles);
  const double length = std::max(1.0e-3, std::min(reach, clear * 0.95));
  return {center.x + direction.x * length, center.y + direction.y * length};
}

Point2D visibility_end(Point2D prev, Point2D center, Point2D next, const Bundle& bundle,
                       const std::vector<Bundle>& bundles) {
  Point2D back{prev.x - center.x, prev.y - center.y};
  Point2D forward{next.x - center.x, next.y - center.y};
  if (std::hypot(forward.x, forward.y) <= 1.0e-12) {
    forward = {-back.x, -back.y};
  }
  if (std::hypot(back.x, back.y) <= 1.0e-12) {
    back = {-forward.x, -forward.y};
  }
  const bool angled = std::hypot(prev.x - center.x, prev.y - center.y) > 1.0e-12 &&
                      std::hypot(next.x - center.x, next.y - center.y) > 1.0e-12;
  const Point2D bisector = rotate_by(unit_of(back), 0.5 * signed_turn(unit_of(back), unit_of(forward)));
  Point2D best = center;
  double best_gap = 1.0e300;
  bool found = false;
  for (const auto& vertex : bundle.ordered_vertices) {
    if (std::hypot(vertex.x - center.x, vertex.y - center.y) <= 1.0e-12) {
      continue;
    }
    if (angled && !on_leq_pi_side(vertex, prev, center, next)) {
      continue;
    }
    const Point2D spoke{vertex.x - center.x, vertex.y - center.y};
    const double gap = std::abs(signed_turn(bisector, spoke));
    if (gap < best_gap) {
      best_gap = gap;
      best = clip_sight(center, vertex, bundles);
      found = true;
    }
  }
  if (found && std::hypot(best.x - center.x, best.y - center.y) > 1.0e-4) {
    return best;
  }
  const double span = std::max(internal::distance(center, prev), internal::distance(center, next));
  const double reach = std::max(0.05, span);
  const double clear = free_reach(center, bisector, reach, bundles);
  const double length = std::max(1.0e-3, std::min(reach, clear * 0.95));
  return {center.x + bisector.x * length, center.y + bisector.y * length};
}

struct Ray {
  Point2D center{};
  Point2D end{};
  std::size_t bundle{0};
};

std::size_t select_cf(const std::vector<Bundle>& bundles) {
  std::size_t selected = 0;
  const Point2D start = bundles.front().concurrent_point;
  for (std::size_t index = 1; index < bundles.size(); ++index) {
    const auto vertices = around_center(bundles[index]);
    const Point2D center = bundles[index].concurrent_point;
    if (vertices.empty()) {
      if (!crosses_obstacle(start, center, bundles)) {
        selected = index;
      }
      continue;
    }
    const Point2D extremes[] = {vertices.front(), vertices.back()};
    const std::size_t count = vertices.size() == 1 ? 1 : 2;
    bool clear = true;
    for (std::size_t end = 0; end < count; ++end) {
      if (triangle_blocked(start, center, extremes[end], bundles)) {
        clear = false;
        break;
      }
    }
    if (clear) {
      selected = index;
    }
  }
  return selected;
}

std::vector<Ray> cstar_rays(const std::vector<Bundle>& bundles, std::size_t first) {
  std::vector<Ray> rays;
  const std::size_t last = bundles.size() - 1;
  for (std::size_t index = first; index <= last; ++index) {
    const Point2D center = bundles[index].concurrent_point;
    const Point2D prev = bundles[index == 0 ? index : index - 1].concurrent_point;
    const Point2D next = bundles[index == last ? index : index + 1].concurrent_point;
    std::vector<Point2D> kept;
    if (!bundles[index].degenerate) {
      for (const auto& vertex : bundles[index].ordered_vertices) {
        if (on_leq_pi_side(vertex, prev, center, next)) {
          kept.push_back(vertex);
        }
      }
    }
    if (kept.empty()) {
      const Point2D end = visibility_end(prev, center, next, bundles[index], bundles);
      if (std::hypot(end.x - center.x, end.y - center.y) > 1.0e-4) {
        rays.push_back({center, end, index});
      }
      continue;
    }
    const Point2D back{prev.x - center.x, prev.y - center.y};
    std::sort(kept.begin(), kept.end(), [&](Point2D left, Point2D right) {
      const Point2D left_spoke{left.x - center.x, left.y - center.y};
      const Point2D right_spoke{right.x - center.x, right.y - center.y};
      return signed_turn(back, left_spoke) < signed_turn(back, right_spoke);
    });
    for (const auto& vertex : kept) {
      rays.push_back({center, vertex, index});
    }
  }

  std::vector<Point2D> trimmed(rays.size());
  for (std::size_t index = 0; index < rays.size(); ++index) {
    double best = 1.0;
    Point2D hit = rays[index].end;
    for (std::size_t other = 0; other < rays.size(); ++other) {
      if (other == index || rays[other].bundle == rays[index].bundle) {
        continue;
      }
      Point2D at{};
      double t_ab = 0.0;
      double t_cd = 0.0;
      if (!proper_hit(rays[index].center, rays[index].end, rays[other].center,
                      rays[other].end, at, t_ab, t_cd)) {
        continue;
      }
      if (t_ab < best) {
        best = t_ab;
        hit = at;
      }
    }
    trimmed[index] = hit;
  }
  for (std::size_t index = 0; index < rays.size(); ++index) {
    rays[index].end = trimmed[index];
  }
  return rays;
}

void append_ring(std::vector<Point2D>& ring, Point2D point) {
  if (!ring.empty() && internal::point_near(ring.back(), point, kWeld)) {
    return;
  }
  ring.push_back(point);
}

std::vector<Point2D> clean_ring(std::vector<Point2D> ring) {
  if (ring.size() >= 2 && internal::point_near(ring.front(), ring.back(), kWeld)) {
    ring.pop_back();
  }
  bool dropped = true;
  while (dropped && ring.size() >= 4) {
    dropped = false;
    for (std::size_t index = 0; index < ring.size(); ++index) {
      const Point2D prev = ring[(index + ring.size() - 1) % ring.size()];
      const Point2D curr = ring[index];
      const Point2D next = ring[(index + 1) % ring.size()];
      if (std::abs(internal::cross(prev, curr, next)) <= 1.0e-10) {
        ring.erase(ring.begin() + static_cast<std::ptrdiff_t>(index));
        dropped = true;
        break;
      }
    }
  }
  if (ring.size() >= 3 && signed_area(ring) < 0.0) {
    std::reverse(ring.begin(), ring.end());
  }
  return ring;
}

bool point_in_ring(Point2D point, const std::vector<Point2D>& ring) {
  bool inside = false;
  for (std::size_t index = 0, prev = ring.size() - 1; index < ring.size(); prev = index++) {
    const Point2D& here = ring[index];
    const Point2D& before = ring[prev];
    const bool straddles = (here.y > point.y) != (before.y > point.y);
    if (!straddles || std::abs(before.y - here.y) <= 1.0e-15) {
      continue;
    }
    const double x_at = (before.x - here.x) * (point.y - here.y) / (before.y - here.y) + here.x;
    if (point.x < x_at) {
      inside = !inside;
    }
  }
  return inside;
}

bool on_boundary(Point2D point, const std::vector<Point2D>& ring) {
  const std::size_t count = ring.size();
  for (std::size_t index = 0; index < count; ++index) {
    if (distance_to_segment(point, ring[index], ring[(index + 1) % count]) <= 1.0e-4) {
      return true;
    }
  }
  return false;
}

bool ring_blocked(const std::vector<Point2D>& ring, const std::vector<Bundle>& bundles) {
  const std::size_t count = ring.size();
  for (std::size_t index = 0; index < count; ++index) {
    if (crosses_obstacle(ring[index], ring[(index + 1) % count], bundles)) {
      return true;
    }
  }
  for (std::size_t i = 0; i < count; ++i) {
    for (std::size_t j = i + 1; j < count; ++j) {
      const bool adjacent = (j == i + 1) || (i == 0 && j + 1 == count);
      if (adjacent) {
        continue;
      }
      Point2D at{};
      double t_ab = 0.0;
      double t_cd = 0.0;
      if (proper_hit(ring[i], ring[(i + 1) % count], ring[j], ring[(j + 1) % count], at,
                     t_ab, t_cd)) {
        return true;
      }
    }
  }
  for (const auto& bundle : bundles) {
    for (const auto& edge : bundle.obstacle_edges) {
      const Point2D mid{(edge.start.x + edge.end.x) * 0.5, (edge.start.y + edge.end.y) * 0.5};
      if (on_boundary(mid, ring)) {
        continue;
      }
      if (point_in_ring(mid, ring)) {
        return true;
      }
    }
  }
  return false;
}

std::optional<std::vector<Triangle>> triangles_from_ring(std::vector<Point2D> raw,
                                                         const std::vector<Bundle>& bundles,
                                                         Point2D start, Point2D goal) {
  const auto ring = clean_ring(std::move(raw));
  if (ring.size() < 3 || std::abs(signed_area(ring)) <= 1.0e-10 || ring_blocked(ring, bundles)) {
    return std::nullopt;
  }
  auto pieces = triangulate(ring);
  if (!pieces) {
    return std::nullopt;
  }
  std::vector<Triangle> clear;
  for (const auto& triangle : *pieces) {
    if (!triangle_blocked(triangle.a, triangle.b, triangle.c, bundles)) {
      clear.push_back(triangle);
    }
  }
  const auto gates = sleeve_portals(clear, start, goal, bundles);
  if (clear.empty() || !gates || gates->empty()) {
    return std::nullopt;
  }
  return clear;
}

std::optional<std::vector<Triangle>> cstar_triangles(const std::vector<Bundle>& bundles) {
  const std::size_t first = select_cf(bundles);
  const auto rays = cstar_rays(bundles, first);
  if (rays.empty()) {
    return std::nullopt;
  }
  std::vector<std::vector<Ray>> fans(bundles.size());
  for (const auto& ray : rays) {
    fans[ray.bundle].push_back(ray);
  }
  const Point2D start = bundles.front().concurrent_point;
  const Point2D goal = bundles.back().concurrent_point;

  std::vector<Point2D> arc;
  std::vector<Point2D> one_side;
  for (std::size_t index = first; index < bundles.size(); ++index) {
    if (fans[index].empty()) {
      continue;
    }
    one_side.push_back(fans[index].front().end);
    for (const auto& ray : fans[index]) {
      arc.push_back(ray.end);
    }
  }

  const Point2D travel{goal.x - start.x, goal.y - start.y};
  const double travel_length2 = travel.x * travel.x + travel.y * travel.y;
  struct Placed {
    double along{0.0};
    Point2D point{};
  };
  std::vector<Placed> left;
  std::vector<Placed> right;
  if (travel_length2 > 1.0e-12) {
    for (const auto& point : arc) {
      const Point2D spoke{point.x - start.x, point.y - start.y};
      const double along = (spoke.x * travel.x + spoke.y * travel.y) / travel_length2;
      const double side = travel.x * spoke.y - travel.y * spoke.x;
      if (along < -0.05 || along > 1.05 || std::abs(side) <= 1.0e-8) {
        continue;
      }
      (side > 0.0 ? left : right).push_back({along, point});
    }
  }
  const auto by_along = [](const Placed& first_point, const Placed& second_point) {
    return first_point.along < second_point.along;
  };
  std::sort(left.begin(), left.end(), by_along);
  std::sort(right.begin(), right.end(), by_along);

  std::vector<Placed> mouth = left;
  mouth.insert(mouth.end(), right.begin(), right.end());
  std::sort(mouth.begin(), mouth.end(), [](const Placed& near_goal, const Placed& far_goal) {
    return near_goal.along > far_goal.along;
  });
  std::vector<Point2D> driven;
  for (const auto& bundle : bundles) {
    append_ring(driven, bundle.concurrent_point);
  }
  std::vector<Point2D> anchors;
  for (const auto& placed : mouth) {
    if (!crosses_obstacle(placed.point, start, bundles)) {
      anchors.push_back(placed.point);
    }
  }
  std::vector<Placed> corner;
  for (const auto& placed : mouth) {
    bool keep = !crosses_obstacle(placed.point, start, bundles);
    if (!keep && !crosses_obstacle(goal, placed.point, bundles)) {
      for (const auto& anchor : anchors) {
        if (internal::distance(placed.point, anchor) <= 0.08 &&
            !crosses_obstacle(placed.point, anchor, bundles)) {
          keep = true;
          break;
        }
      }
    }
    if (keep) {
      corner.push_back(placed);
    }
  }
  std::sort(corner.begin(), corner.end(), [](const Placed& near_goal, const Placed& far_goal) {
    return near_goal.along > far_goal.along;
  });
  std::vector<Point2D> opposite;
  Point2D cursor = goal;
  for (const auto& placed : corner) {
    if (!crosses_obstacle(cursor, start, bundles)) {
      break;
    }
    if (crosses_obstacle(cursor, placed.point, bundles)) {
      continue;
    }
    const Point2D previous = opposite.size() < 2 ? goal : opposite[opposite.size() - 2];
    if (!opposite.empty() && triangle_blocked(previous, cursor, placed.point, bundles)) {
      continue;
    }
    opposite.push_back(placed.point);
    cursor = placed.point;
  }
  if (opposite.size() >= 2 && !crosses_obstacle(cursor, start, bundles) && driven.size() >= 2) {
    for (std::size_t index = driven.size() - 1; index > 0; --index) {
      const Point2D bridge{(driven[index - 1].x + driven[index].x) * 0.5,
                           (driven[index - 1].y + driven[index].y) * 0.5};
      std::vector<Point2D> homotopy;
      append_ring(homotopy, start);
      for (auto point = opposite.rbegin(); point != opposite.rend(); ++point) {
        append_ring(homotopy, *point);
      }
      append_ring(homotopy, goal);
      append_ring(homotopy, bridge);
      if (auto homotopy_triangles = triangles_from_ring(homotopy, bundles, start, goal)) {
        return homotopy_triangles;
      }
    }
  }

  std::vector<Point2D> channel;
  append_ring(channel, start);
  for (const auto& placed : left) {
    append_ring(channel, placed.point);
  }
  append_ring(channel, goal);
  for (auto placed = right.rbegin(); placed != right.rend(); ++placed) {
    append_ring(channel, placed->point);
  }
  std::vector<Point2D> chain;
  append_ring(chain, start);
  for (const auto& point : arc) {
    append_ring(chain, point);
  }
  append_ring(chain, goal);

  std::vector<Point2D> spine;
  append_ring(spine, start);
  for (const auto& point : one_side) {
    append_ring(spine, point);
  }
  append_ring(spine, goal);
  if (auto channel_triangles = triangles_from_ring(channel, bundles, start, goal)) {
    return channel_triangles;
  }
  if (auto arc_triangles = triangles_from_ring(chain, bundles, start, goal)) {
    return arc_triangles;
  }
  return triangles_from_ring(spine, bundles, start, goal);
}

}  // namespace

GatePreparationResult prepare_gates(const BundleSequence& sequence,
                                    bool gate_contract_enabled,
                                    double linear_epsilon) {
  if (!std::isfinite(linear_epsilon) || linear_epsilon < 0.0) {
    throw std::invalid_argument("linear_epsilon must be finite and non-negative");
  }
  if (!gate_contract_enabled) {
    return {false, GateFailureReason::ContractNotEnabled,
            "The Module 3 portal contract has not been agreed; no gates were manufactured.",
            {}};
  }
  if (sequence.bundles.size() < 2) {
    return {false, GateFailureReason::EmptySequence,
            "A gate sequence needs at least two bundles.", {}};
  }
  for (const auto& bundle : sequence.bundles) {
    internal::require_finite(bundle.concurrent_point, "bundle center");
    for (const auto& vertex : bundle.ordered_vertices) {
      internal::require_finite(vertex, "bundle vertex");
    }
  }

  const auto triangles = cstar_triangles(sequence.bundles);
  if (!triangles) {
    return {false, GateFailureReason::UnsupportedGeometry,
            "The preprocessed bundle sequence C* has no corridor triangles.", {}};
  }
  auto gates = sleeve_portals(*triangles, sequence.bundles.front().concurrent_point,
                              sequence.bundles.back().concurrent_point,
                              sequence.bundles);
  if (!gates) {
    return {false, GateFailureReason::UnsupportedGeometry,
            "The triangulated corridor has no sleeve from the stuck end to the return end.",
            {}};
  }
  return {true, GateFailureReason::None, {}, std::move(*gates)};
}

}  // namespace mars::graph_bundle_management
