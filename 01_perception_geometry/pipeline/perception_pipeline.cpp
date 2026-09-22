#include <mars/perception_geometry/perception_pipeline.hpp>
#include <mars/perception_geometry/neighbor_sight.hpp>
#include <mars/perception_geometry/closed_sights.hpp>
#include <mars/perception_geometry/open_sights.hpp>
#include <mars/perception_geometry/open_points.hpp>

namespace mars::perception_geometry {
PerceptionResult perceive(const Point2D& center,double radius,
                          const std::vector<Polygon2D>& obstacles) {
    PerceptionResult result;
    result.neighbor_sight=compute_neighbor_sight(center,radius,obstacles);
    result.closed_sights=compute_closed_sights(result.neighbor_sight);
    result.open_sights=compute_open_sights(result.closed_sights);
    result.open_points=compute_open_points(center,radius,result.open_sights);
    return result;
}
}  // namespace mars::perception_geometry
