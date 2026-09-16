#pragma once

#include <mars/common/geometry_types.hpp>
#include <cstddef>
#include <optional>
#include <vector>

namespace mars::perception_geometry {
using common::Point2D;
using common::Segment2D;
using common::Polygon2D;
using common::Circle2D;
using common::Pose2D;
using common::AngularInterval;
using common::pi;
using common::two_pi;

struct NeighborSight {
    Point2D center;
    double radius{0.0};
    // Occlusion-resolved fragments, not all clipped obstacle edges.
    std::vector<Segment2D> visible_boundaries;
};
struct ClosedSight {
    AngularInterval interval;
    // A merged sector may be supported by several visible fragments.
    std::vector<Segment2D> visible_boundaries;
};
struct OpenSight { AngularInterval interval; };
struct OpenPoint {
    Point2D point;
    double angle{0.0};
    std::optional<std::size_t> sight_index;
};
struct PerceptionResult {
    NeighborSight neighbor_sight;
    std::vector<ClosedSight> closed_sights;
    std::vector<OpenSight> open_sights;
    std::vector<OpenPoint> open_points;
};
}  // namespace mars::perception_geometry
