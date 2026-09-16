#include <mars/perception_geometry/open_points.hpp>
#include <mars/perception_geometry/geometry.hpp>

namespace mars::perception_geometry {
OpenPoint compute_open_point(const Point2D& center,double radius,const OpenSight& sight,
                             std::optional<std::size_t> sight_index) {
    const double angle=angular_midpoint(sight.interval);
    return {point_on_circle({center,radius},angle),angle,sight_index};
}
std::vector<OpenPoint> compute_open_points(const Point2D& center,double radius,
                                         const std::vector<OpenSight>& sights) {
    validate_circle({center,radius});
    std::vector<OpenPoint> result;
    result.reserve(sights.size());
    for (std::size_t i=0;i<sights.size();++i)
        result.push_back(compute_open_point(center,radius,sights[i],i));
    return result;
}
}  // namespace mars::perception_geometry
