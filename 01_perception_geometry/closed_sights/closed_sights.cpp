#include <mars/perception_geometry/closed_sights.hpp>
#include "detail.hpp"
#include "intervals.hpp"
#include <stdexcept>

namespace mars::perception_geometry {
std::vector<ClosedSight> compute_closed_sights(const NeighborSight& neighbor) {
    validate_circle({neighbor.center,neighbor.radius});
    std::vector<AngularInterval> projections;
    std::vector<Segment2D> boundaries;
    for (const auto& segment:neighbor.visible_boundaries) {
        validate_point(segment.start); validate_point(segment.end);
        if (distance(segment.start,neighbor.center)>neighbor.radius+linear_epsilon ||
            distance(segment.end,neighbor.center)>neighbor.radius+linear_epsilon)
            throw std::invalid_argument("visible boundary outside sensing circle");
        if (distance(segment.start,segment.end)<=linear_epsilon) continue;
        if (detail::on_segment(neighbor.center,segment))
            throw std::invalid_argument("visible boundary passes through observer");
        const Point2D a{segment.start.x-neighbor.center.x,segment.start.y-neighbor.center.y};
        const Point2D b{segment.end.x-neighbor.center.x,segment.end.y-neighbor.center.y};
        double start=detail::angle(a);
        double sweep=normalize_angle(detail::angle(b)-start);
        if (sweep>pi) { start=detail::angle(b); sweep=two_pi-sweep; }
        if (sweep<=angular_epsilon) continue;
        projections.push_back({start,sweep});
        boundaries.push_back(segment);
    }
    std::vector<ClosedSight> result;
    for (const auto& interval:detail::circular_union(projections)) {
        ClosedSight sight{interval,{}};
        for (std::size_t i=0;i<projections.size();++i)
            if (detail::contains(interval,angular_midpoint(projections[i])))
                sight.visible_boundaries.push_back(boundaries[i]);
        result.push_back(std::move(sight));
    }
    return result;
}
}  // namespace mars::perception_geometry
