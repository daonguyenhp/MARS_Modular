#include <mars/perception_geometry/neighbor_sight.hpp>
#include "detail.hpp"
#include <stdexcept>

namespace mars::perception_geometry {
namespace {
std::vector<Segment2D> local_edges(const Point2D& center,
                                 const std::vector<Polygon2D>& obstacles) {
    std::vector<Segment2D> result;
    for (const auto& polygon : obstacles) {
        std::vector<Point2D> points;
        for (const auto& p : polygon.vertices) {
            validate_point(p);
            Point2D local{p.x-center.x,p.y-center.y};
            if (!std::isfinite(local.x) || !std::isfinite(local.y))
                throw std::overflow_error("observer-relative coordinate overflow");
            if (points.empty() || distance(points.back(),local)>linear_epsilon)
                points.push_back(local);
        }
        if (points.size()>1 && distance(points.front(),points.back())<=linear_epsilon)
            points.pop_back();
        if (points.size()<2) continue;
        const std::size_t count=points.size()==2 ? 1 : points.size();
        std::vector<Segment2D> edges;
        for (std::size_t i=0;i<count;++i) {
            const Segment2D edge{points[i],points[(i+1)%points.size()]};
            if (detail::on_segment({0,0},edge))
                throw std::invalid_argument("observer lies on an obstacle boundary");
            edges.push_back(edge);
        }
        if (points.size()>2) {
            detail::Real area=0;
            bool inside=false;
            for (std::size_t i=0;i<edges.size();++i) {
                const auto& a=edges[i].start; const auto& b=edges[i].end;
                area+=detail::cross(a.x,a.y,b.x,b.y);
                if ((a.y>0)!=(b.y>0)) {
                    const detail::Real x=detail::Real(a.x)-detail::Real(a.y)*
                        (detail::Real(b.x)-a.x)/(detail::Real(b.y)-a.y);
                    if (x>0) inside=!inside;
                }
                // Adjacent edges may meet at their shared endpoint only.
                const auto& next=edges[(i+1)%edges.size()];
                if (detail::on_segment(a,next) || detail::on_segment(next.end,edges[i]))
                    throw std::invalid_argument("polygon backtracks along an edge");
                for (std::size_t j=i+2;j<edges.size();++j) {
                    if (i==0 && j+1==edges.size()) continue;
                    if (detail::touches(edges[i],edges[j]))
                        throw std::invalid_argument("polygon must be simple");
                }
            }
            if (std::abs(area)<=linear_epsilon*linear_epsilon)
                throw std::invalid_argument("polygon has no area; use a two-vertex wall");
            if (inside) throw std::invalid_argument("observer lies inside a filled obstacle");
        }
        result.insert(result.end(),edges.begin(),edges.end());
    }
    return result;
}
struct Fragment { std::size_t source; Segment2D segment; };
}  // namespace

NeighborSight compute_neighbor_sight(const Point2D& center, double radius,
                                     const std::vector<Polygon2D>& obstacles) {
    validate_circle({center,radius});
    std::vector<Segment2D> edges;
    for (const auto& edge : local_edges(center,obstacles)) {
        const auto clipped=clip_segment_to_circle(edge,{{0,0},radius});
        if (clipped && distance(clipped->start,clipped->end)>linear_epsilon)
            edges.push_back(*clipped);
    }
    // All endpoint and order-exchange events. Between events, the nearest
    // intersected edge is constant, so one interior ray identifies it exactly.
    std::vector<double> angles{0,two_pi};
    for (std::size_t i=0;i<edges.size();++i) {
        angles.push_back(detail::angle(edges[i].start));
        angles.push_back(detail::angle(edges[i].end));
        for (std::size_t j=i+1;j<edges.size();++j)
            if (const auto p=detail::crossing(edges[i],edges[j])) angles.push_back(detail::angle(*p));
    }
    std::sort(angles.begin(),angles.end());
    // Keep distinct floating-point events; only positive cells wider than the
    // documented angular resolution are considered below.
    angles.erase(std::unique(angles.begin(),angles.end()),angles.end());
    std::vector<Fragment> visible;
    for (std::size_t k=1;k<angles.size();++k) {
        const double start=angles[k-1], end=angles[k];
        if (end-start<=angular_epsilon) continue;
        const double mid=start+(end-start)/2;
        std::optional<std::size_t> nearest;
        double best=std::numeric_limits<double>::infinity();
        for (std::size_t j=0;j<edges.size();++j) {
            if (const auto hit=ray_segment_intersection({0,0},mid,edges[j])) {
                const double d=distance({0,0},*hit);
                if (d<best) { best=d; nearest=j; }
            }
        }
        if (!nearest) continue;
        const auto first=ray_segment_intersection({0,0},start,edges[*nearest]);
        const auto last=ray_segment_intersection({0,0},end,edges[*nearest]);
        if (!first || !last)
            throw std::runtime_error("visibility event intersection lost numerical precision");
        if (distance(*first,*last)<=linear_epsilon) continue;
        if (!visible.empty() && visible.back().source==*nearest &&
            distance(visible.back().segment.end,*first)<=linear_epsilon)
            visible.back().segment.end=*last;
        else visible.push_back({*nearest,{*first,*last}});
    }
    // Rejoin a single visible edge split by the artificial zero-angle seam.
    if (visible.size()>1 && visible.front().source==visible.back().source &&
        distance(visible.back().segment.end,visible.front().segment.start)<=linear_epsilon) {
        visible.front().segment.start=visible.back().segment.start;
        visible.pop_back();
    }
    std::sort(visible.begin(),visible.end(),[](const Fragment& a,const Fragment& b) {
        return detail::angle(a.segment.start)<detail::angle(b.segment.start);
    });
    NeighborSight result{center,radius,{}};
    for (const auto& f : visible) {
        const Segment2D global{{f.segment.start.x+center.x,f.segment.start.y+center.y},
                               {f.segment.end.x+center.x,f.segment.end.y+center.y}};
        if (!std::isfinite(global.start.x) || !std::isfinite(global.start.y) ||
            !std::isfinite(global.end.x) || !std::isfinite(global.end.y))
            throw std::overflow_error("visible boundary coordinate overflow");
        result.visible_boundaries.push_back(global);
    }
    return result;
}
}  // namespace mars::perception_geometry
