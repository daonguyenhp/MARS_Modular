#include <mars/perception_geometry/open_points.hpp>
#include <mars/perception_geometry/geometry.hpp>

namespace mars::perception_geometry {
namespace {

OpenPoint make_open_point(const Point2D& center, double radius, double angle,
                          std::optional<std::size_t> sight_index) {
    angle = normalize_angle(angle);
    return {point_on_circle({center, radius}, angle), angle, sight_index};
}
}  // namespace

OpenPoint compute_open_point(const Point2D& center,double radius,const OpenSight& sight,
                             std::optional<std::size_t> sight_index) {
    const double angle=angular_midpoint(sight.interval);
    return make_open_point(center, radius, angle, sight_index);
}
std::vector<OpenPoint> compute_open_points(const Point2D& center,double radius,
                                         const std::vector<OpenSight>& sights) {
    validate_circle({center,radius});
    std::vector<OpenPoint> result;
    result.reserve(sights.size());
    for (std::size_t i=0;i<sights.size();++i) {
        // One point per open sight: the middle of the free arc. Samples toward
        // the arc edges sit against the wall and are not a way through.
        result.push_back(compute_open_point(center, radius, sights[i], i));
    }
    return result;
}
}  // namespace mars::perception_geometry
