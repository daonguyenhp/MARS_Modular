#include <mars/perception_geometry/geometry.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace mars::perception_geometry {
namespace {
using Real = long double;
// Ray directions come from double-precision trigonometry, including sin(pi).
constexpr Real parallel_epsilon = 64 * std::numeric_limits<double>::epsilon();
Point2D checked(Real x, Real y) {
    Point2D p{static_cast<double>(x), static_cast<double>(y)};
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
        throw std::overflow_error("geometry result is not representable");
    return p;
}
struct CircleRoots {
    Real length, ux, uy, first, last;
    bool intersects;
};
CircleRoots roots(const Segment2D& s, const Circle2D& c) {
    validate_point(s.start); validate_point(s.end); validate_circle(c);
    const Real dx = Real(s.end.x) - s.start.x, dy = Real(s.end.y) - s.start.y;
    const Real length = std::hypot(dx, dy);
    if (length <= linear_epsilon) return {length, 0, 0, 0, 0, false};
    const Real ux = dx / length, uy = dy / length;
    const Real x = Real(s.start.x) - c.center.x, y = Real(s.start.y) - c.center.y;
    const Real projection = -(x * ux + y * uy);
    const Real perpendicular = std::abs(x * uy - y * ux);
    if (perpendicular > c.radius) return {length, ux, uy, 0, 0, false};
    const Real half = std::sqrt(std::max(Real(0),
        (Real(c.radius) - perpendicular) * (Real(c.radius) + perpendicular)));
    return {length, ux, uy, projection - half, projection + half, true};
}
}  // namespace

void validate_point(const Point2D& p) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y))
        throw std::invalid_argument("point must be finite");
}
void validate_circle(const Circle2D& c) {
    validate_point(c.center);
    if (!std::isfinite(c.radius) || c.radius <= linear_epsilon)
        throw std::invalid_argument("radius must exceed linear_epsilon");
}
void validate_interval(const AngularInterval& i) {
    if (!std::isfinite(i.start) || !std::isfinite(i.sweep) ||
        i.start < 0 || i.start >= two_pi || i.sweep < 0 || i.sweep > two_pi)
        throw std::invalid_argument("invalid start/sweep interval");
}
double distance(const Point2D& a, const Point2D& b) {
    validate_point(a); validate_point(b);
    const double d = static_cast<double>(std::hypot(Real(a.x)-b.x, Real(a.y)-b.y));
    if (!std::isfinite(d)) throw std::overflow_error("distance overflow");
    return d;
}
double normalize_angle(double angle) {
    if (!std::isfinite(angle)) throw std::invalid_argument("angle must be finite");
    double value = std::fmod(angle, two_pi);
    if (value < 0) value += two_pi;
    return value == 0 || value >= two_pi ? 0.0 : value;
}
double angular_midpoint(const AngularInterval& i) {
    validate_interval(i);
    if (i.sweep == 0) throw std::invalid_argument("empty interval has no midpoint");
    return normalize_angle(i.start + i.sweep / 2);
}
Point2D point_on_circle(const Circle2D& c, double angle) {
    validate_circle(c);
    angle = normalize_angle(angle);
    return checked(Real(c.center.x) + Real(c.radius) * std::cos(angle),
                   Real(c.center.y) + Real(c.radius) * std::sin(angle));
}
std::optional<Point2D> ray_segment_intersection(
    const Point2D& o, double angle, const Segment2D& s) {
    validate_point(o); validate_point(s.start); validate_point(s.end);
    angle = normalize_angle(angle);
    const Real ux = std::cos(angle), uy = std::sin(angle);
    const Real dx = Real(s.end.x)-s.start.x, dy = Real(s.end.y)-s.start.y;
    const Real length = std::hypot(dx, dy);
    const Real x = Real(s.start.x)-o.x, y = Real(s.start.y)-o.y;
    const Real projection = x*ux + y*uy;
    if (length <= linear_epsilon) {
        if (std::abs(x*uy-y*ux) <= linear_epsilon && projection >= 0) return s.start;
        return std::nullopt;
    }
    const Real vx = dx/length, vy = dy/length;
    const Real det = ux*vy-uy*vx;
    if (std::abs(det) <= parallel_epsilon) {
        if (std::abs(x*uy-y*ux) > linear_epsilon) return std::nullopt;
        const Real other = projection + dx*ux + dy*uy;
        if (std::max(projection, other) < 0) return std::nullopt;
        const Real t = std::max(Real(0), std::min(projection, other));
        return checked(Real(o.x)+t*ux, Real(o.y)+t*uy);
    }
    const Real t = (x*vy-y*vx)/det;
    const Real along = (x*uy-y*ux)/det;
    if (t < -linear_epsilon || along < -linear_epsilon || along > length+linear_epsilon)
        return std::nullopt;
    const Real bounded = std::clamp(along, Real(0), length);
    return checked(Real(s.start.x)+bounded*vx, Real(s.start.y)+bounded*vy);
}
std::vector<Point2D> segment_circle_intersections(const Segment2D& s, const Circle2D& c) {
    const auto r = roots(s, c);
    if (r.length <= linear_epsilon) {
        if (std::abs(distance(s.start, c.center)-c.radius) <= linear_epsilon) return {s.start};
        return {};
    }
    std::vector<Point2D> result;
    if (!r.intersects) return result;
    for (const Real t : {r.first, r.last}) {
        if (t < -linear_epsilon || t > r.length+linear_epsilon) continue;
        const Real bounded = std::clamp(t, Real(0), r.length);
        const auto p = checked(Real(s.start.x)+bounded*r.ux, Real(s.start.y)+bounded*r.uy);
        if (result.empty() || distance(result.back(), p) > linear_epsilon) result.push_back(p);
    }
    return result;
}
std::optional<Segment2D> clip_segment_to_circle(const Segment2D& s, const Circle2D& c) {
    const auto r = roots(s, c);
    if (r.length <= linear_epsilon) {
        if (distance(s.start, c.center) <= c.radius) return Segment2D{s.start,s.start};
        return std::nullopt;
    }
    if (!r.intersects) return std::nullopt;
    const Real a = std::max(Real(0), r.first), b = std::min(r.length, r.last);
    if (b < a) return std::nullopt;
    return Segment2D{checked(Real(s.start.x)+a*r.ux, Real(s.start.y)+a*r.uy),
                     checked(Real(s.start.x)+b*r.ux, Real(s.start.y)+b*r.uy)};
}
}  // namespace mars::perception_geometry
