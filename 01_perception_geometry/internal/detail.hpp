#pragma once
#include <mars/perception_geometry/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

namespace mars::perception_geometry::detail {
using Real = long double;
inline Real cross(Real x, Real y, Real u, Real v) { return x*v-y*u; }
inline double angle(const Point2D& p) { return normalize_angle(std::atan2(p.y,p.x)); }
inline bool on_segment(const Point2D& p, const Segment2D& s) {
    const Real dx=Real(s.end.x)-s.start.x, dy=Real(s.end.y)-s.start.y;
    const Real length=std::hypot(dx,dy);
    if (length <= linear_epsilon) return distance(p,s.start) <= linear_epsilon;
    const Real x=Real(p.x)-s.start.x, y=Real(p.y)-s.start.y;
    const Real t=(x*dx+y*dy)/length;
    return std::abs(cross(x,y,dx/length,dy/length)) <= linear_epsilon &&
           t >= -linear_epsilon && t <= length+linear_epsilon;
}
// Unique intersection. Collinear overlap has only endpoint visibility events.
inline std::optional<Point2D> crossing(const Segment2D& a, const Segment2D& b) {
    const Real ax=Real(a.end.x)-a.start.x, ay=Real(a.end.y)-a.start.y;
    const Real bx=Real(b.end.x)-b.start.x, by=Real(b.end.y)-b.start.y;
    const Real la=std::hypot(ax,ay), lb=std::hypot(bx,by);
    if (la <= linear_epsilon || lb <= linear_epsilon) return std::nullopt;
    const Real ux=ax/la, uy=ay/la, vx=bx/lb, vy=by/lb;
    const Real det=cross(ux,uy,vx,vy);
    if (std::abs(det) <= 64*std::numeric_limits<Real>::epsilon()) return std::nullopt;
    const Real x=Real(b.start.x)-a.start.x, y=Real(b.start.y)-a.start.y;
    const Real t=cross(x,y,vx,vy)/det, q=cross(x,y,ux,uy)/det;
    if (t < -linear_epsilon || t > la+linear_epsilon ||
        q < -linear_epsilon || q > lb+linear_epsilon) return std::nullopt;
    const Real bounded=std::clamp(t,Real(0),la);
    return Point2D{double(Real(a.start.x)+bounded*ux), double(Real(a.start.y)+bounded*uy)};
}
inline bool touches(const Segment2D& a, const Segment2D& b) {
    return crossing(a,b).has_value() || on_segment(a.start,b) || on_segment(a.end,b) ||
           on_segment(b.start,a) || on_segment(b.end,a);
}
}  // namespace mars::perception_geometry::detail
