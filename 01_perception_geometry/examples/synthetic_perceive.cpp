#include <iostream>
#include <vector>

#include <mars/perception_geometry/perception_pipeline.hpp>

// MD4.4: short Module 1 example — no ROS, no Module 2/3.
int main() {
  using namespace mars::perception_geometry;

  const Point2D center{0.0, 0.0};
  const double radius = 5.0;
  // U-shaped walls: one closed sight, one open mouth with an open point.
  const std::vector<Polygon2D> obstacles{
      {{{-2.0, -2.0}, {2.0, -2.0}}},
      {{{2.0, -2.0}, {2.0, 2.0}}},
      {{{2.0, 2.0}, {-2.0, 2.0}}},
  };

  const PerceptionResult result = perceive(center, radius, obstacles);

  std::cout << "closed=" << result.closed_sights.size()
            << " open=" << result.open_sights.size()
            << " open_points=" << result.open_points.size() << "\n";
  for (const auto& open : result.open_points) {
    std::cout << "  open_point=(" << open.point.x << ", " << open.point.y
              << ") angle=" << open.angle << "\n";
  }
  return 0;
}
