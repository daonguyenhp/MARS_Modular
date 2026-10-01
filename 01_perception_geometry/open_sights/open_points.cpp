#include <mars/perception_geometry/open_points.hpp>
#include <mars/perception_geometry/geometry.hpp>

namespace mars::perception_geometry {
namespace {
// Keep the paper midpoint, then sample a wide mouth. 0.45 rad is about 0.34 m
// along the Mode 1 circle (r = 0.75 m), far enough that Module 2 does not merge them.
constexpr double kSampleStep = 0.45;
constexpr double kEdgeMargin = 0.05;

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
    for (std::size_t i=0;i<sights.size();++i) {
        const AngularInterval& interval = sights[i].interval;
        const double mid = angular_midpoint(interval);
        result.push_back(make_open_point(center, radius, mid, i));
        const double limit = interval.sweep / 2.0 - kEdgeMargin;
        for (int step = 1; static_cast<double>(step) * kSampleStep < limit; ++step) {
            const double offset = static_cast<double>(step) * kSampleStep;
            result.push_back(make_open_point(center, radius, mid + offset, i));
            result.push_back(make_open_point(center, radius, mid - offset, i));
        }
    }
    return result;
}
}  // namespace mars::perception_geometry
