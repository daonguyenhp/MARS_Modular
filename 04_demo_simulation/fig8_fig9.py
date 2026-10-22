"""Figure 8 and Figure 9. From MARS_Modular/04_demo_simulation: python fig8_fig9.py"""

from __future__ import annotations


import math
from typing import Iterable, Optional, Sequence

Point = tuple[float, float]
Segment = tuple[Point, Point]
Polygon = list[Point]

EPS = 1e-9


def add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def mul(a: Point, s: float) -> Point:
    return (a[0] * s, a[1] * s)


def dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def cross(a: Point, b: Point) -> float:
    return a[0] * b[1] - a[1] * b[0]


def norm(a: Point) -> float:
    return math.hypot(a[0], a[1])


def dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def lerp(a: Point, b: Point, t: float) -> Point:
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def unit(a: Point) -> Point:
    length = norm(a)
    if length <= EPS:
        return (0.0, 0.0)
    return (a[0] / length, a[1] / length)


def angle_of(a: Point) -> float:
    return math.atan2(a[1], a[0])


def wrap_pi(theta: float) -> float:
    while theta <= -math.pi:
        theta += 2.0 * math.pi
    while theta > math.pi:
        theta -= 2.0 * math.pi
    return theta


def wrap_2pi(theta: float) -> float:
    theta = math.fmod(theta, 2.0 * math.pi)
    if theta < 0.0:
        theta += 2.0 * math.pi
    return theta


def angle_between(a: Point, b: Point) -> float:
    """Unsigned angle in [0, pi] between two vectors."""
    na, nb = norm(a), norm(b)
    if na <= EPS or nb <= EPS:
        return 0.0
    cosine = max(-1.0, min(1.0, dot(a, b) / (na * nb)))
    return math.acos(cosine)


def orient(a: Point, b: Point, c: Point) -> float:
    return cross(sub(b, a), sub(c, a))


def on_segment(a: Point, b: Point, p: Point, eps: float = 1e-8) -> bool:
    if abs(orient(a, b, p)) > eps:
        return False
    return (min(a[0], b[0]) - eps <= p[0] <= max(a[0], b[0]) + eps and
            min(a[1], b[1]) - eps <= p[1] <= max(a[1], b[1]) + eps)


def segments_properly_intersect(a: Point, b: Point, c: Point, d: Point,
                                eps: float = 1e-8) -> bool:
    """True when the open segments overlap in their interiors."""
    o1, o2 = orient(a, b, c), orient(a, b, d)
    o3, o4 = orient(c, d, a), orient(c, d, b)
    if abs(o1) <= eps or abs(o2) <= eps or abs(o3) <= eps or abs(o4) <= eps:
        return False
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def segment_intersection(a: Point, b: Point, c: Point, d: Point,
                         eps: float = 1e-9) -> Optional[Point]:
    """Intersection of closed segments, or None."""
    r, s = sub(b, a), sub(d, c)
    den = cross(r, s)
    if abs(den) <= eps:
        return None
    t = cross(sub(c, a), s) / den
    u = cross(sub(c, a), r) / den
    if -eps <= t <= 1.0 + eps and -eps <= u <= 1.0 + eps:
        return lerp(a, b, max(0.0, min(1.0, t)))
    return None


def ray_segment_hit(origin: Point, direction: Point, a: Point, b: Point,
                    eps: float = 1e-9) -> Optional[float]:
    """Distance along `direction` to segment ab. Direction need not be a unit."""
    vx, vy = direction
    sx, sy = b[0] - a[0], b[1] - a[1]
    den = vx * sy - vy * sx
    if abs(den) <= eps:
        return None
    dx, dy = a[0] - origin[0], a[1] - origin[1]
    t = (dx * sy - dy * sx) / den
    u = (dx * vy - dy * vx) / den
    if t > eps and -eps <= u <= 1.0 + eps:
        return t
    return None


def path_length(points: Sequence[Point]) -> float:
    total = 0.0
    for i in range(1, len(points)):
        total += dist(points[i - 1], points[i])
    return total


def point_in_polygon(point: Point, polygon: Polygon, eps: float = 1e-9) -> bool:
    """Even-odd test. Boundary counts as inside."""
    x, y = point
    for i in range(len(polygon)):
        a, b = polygon[i], polygon[(i + 1) % len(polygon)]
        if on_segment(a, b, point, eps=1e-7):
            return True
    inside = False
    j = len(polygon) - 1
    for i in range(len(polygon)):
        yi, yj = polygon[i][1], polygon[j][1]
        xi, xj = polygon[i][0], polygon[j][0]
        if (yi > y) != (yj > y):
            xint = (xj - xi) * (y - yi) / (yj - yi + 0.0) + xi
            if x < xint:
                inside = not inside
        j = i
    return inside


def clearance_to_obstacles(point: Point, obstacles: Sequence[Polygon]) -> float:
    """Distance from a point to the nearest obstacle edge."""
    best = float("inf")
    for polygon in obstacles:
        count = len(polygon)
        for index in range(count):
            best = min(best, _distance_to_segment(
                point, polygon[index], polygon[(index + 1) % count]))
            if point_in_polygon(point, polygon):
                return 0.0
    return best


def _distance_to_segment(point: Point, start: Point, end: Point) -> float:
    vx, vy = end[0] - start[0], end[1] - start[1]
    length2 = vx * vx + vy * vy
    if length2 <= EPS:
        return dist(point, start)
    scale = ((point[0] - start[0]) * vx + (point[1] - start[1]) * vy) / length2
    scale = max(0.0, min(1.0, scale))
    return dist(point, (start[0] + scale * vx, start[1] + scale * vy))


def segment_clearance(a: Point, b: Point, obstacles: Sequence[Polygon],
                      samples: int = 16) -> float:
    best = min(clearance_to_obstacles(a, obstacles),
               clearance_to_obstacles(b, obstacles))
    for index in range(1, samples):
        scale = index / samples
        point = (a[0] + (b[0] - a[0]) * scale, a[1] + (b[1] - a[1]) * scale)
        best = min(best, clearance_to_obstacles(point, obstacles))
    return best


def pull_clear(origin: Point, point: Point, obstacles: Sequence[Polygon],
               margin: float) -> Point:
    """Walk from `point` back toward `origin` until the pose and the
    segment leading to it both clear the walls."""
    def ok(candidate: Point) -> bool:
        return (clearance_to_obstacles(candidate, obstacles) >= margin
                and segment_clearance(origin, candidate, obstacles) >= margin)

    if ok(point):
        return point
    kept = origin
    low, high = 0.0, 1.0
    for _ in range(20):
        mid = 0.5 * (low + high)
        candidate = lerp(origin, point, mid)
        if ok(candidate):
            kept = candidate
            low = mid
        else:
            high = mid
    return kept


def segment_hits_obstacles(a: Point, b: Point, obstacles: Sequence[Polygon],
                           eps: float = 1e-8) -> bool:
    if dist(a, b) <= eps:
        return False
    for polygon in obstacles:
        n = len(polygon)
        for i in range(n):
            c = polygon[i]
            d = polygon[(i + 1) % n]
            if segments_properly_intersect(a, b, c, d, eps):
                return True
    return False


def first_obstacle_hit(origin: Point, direction: Point,
                       obstacles: Sequence[Polygon],
                       max_distance: float) -> tuple[float, Point]:
    """Nearest obstacle along the ray, capped at max_distance."""
    best = max_distance
    step = unit(direction)
    if norm(step) <= EPS:
        return max_distance, origin
    for polygon in obstacles:
        n = len(polygon)
        for i in range(n):
            hit = ray_segment_hit(origin, step, polygon[i], polygon[(i + 1) % n])
            if hit is not None and EPS < hit < best:
                best = hit
    return best, add(origin, mul(step, best))


def visible_point(origin: Point, target: Point, obstacles: Sequence[Polygon],
                  radius: float) -> bool:
    gap = dist(origin, target)
    if gap > radius + 1e-6 or gap <= EPS:
        return False
    return not segment_hits_obstacles(origin, target, obstacles)


def polygon_vertices(obstacles: Iterable[Polygon]) -> list[Point]:
    found: list[Point] = []
    for polygon in obstacles:
        for vertex in polygon:
            if all(dist(vertex, old) > 1e-6 for old in found):
                found.append(vertex)
    return found


def rect(x0: float, y0: float, x1: float, y1: float) -> Polygon:
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

import math


INF = math.inf


def ranking_parts(point: Point, current: Point, goal: Point,
                  alpha: float = 1.0, beta: float = 1.0,
                  linear_eps: float = 1e-8,
                  angular_eps: float = 1e-8) -> tuple[float, float, float]:
    """Return (distance to goal, angle at the robot, score)."""
    distance = dist(point, goal)
    turn = angle_between(sub(point, current), sub(goal, current))
    if distance <= linear_eps or turn <= angular_eps:
        return distance, turn, INF
    return distance, turn, alpha / distance + beta / turn


def ranking_score(point: Point, current: Point, goal: Point,
                  alpha: float = 1.0, beta: float = 1.0,
                  linear_eps: float = 1e-8,
                  angular_eps: float = 1e-8) -> float:
    return ranking_parts(point, current, goal, alpha, beta,
                         linear_eps, angular_eps)[2]

import math
from dataclasses import dataclass, field



@dataclass
class SightScan:
    center: Point
    radius: float
    samples: list[tuple[float, float, Point]] = field(default_factory=list)
    open_arcs: list[tuple[float, float]] = field(default_factory=list)
    closed_arcs: list[tuple[float, float]] = field(default_factory=list)
    open_points: list[Point] = field(default_factory=list)
    bundle_endpoints: list[Point] = field(default_factory=list)

    def radius_at(self, theta: float) -> float:
        if not self.samples:
            return 0.0
        theta = wrap_2pi(theta)
        angles = [sample[0] for sample in self.samples]
        # samples are sorted and cover a full turn; the last wraps to the first
        hi = 0
        for i, angle in enumerate(angles):
            if angle >= theta:
                hi = i
                break
        else:
            hi = 0
        lo = (hi - 1) % len(angles)
        a0, d0 = angles[lo], self.samples[lo][1]
        a1, d1 = angles[hi], self.samples[hi][1]
        span = (a1 - a0) % (2.0 * math.pi)
        if span <= 1e-12:
            return d0
        shift = (theta - a0) % (2.0 * math.pi)
        weight = shift / span
        return d0 + (d1 - d0) * weight

    def contains_interior(self, point: Point, margin: float) -> bool:
        """True when `point` is strictly inside this visible disk."""
        gap = dist(self.center, point)
        if gap >= self.radius - margin:
            return False
        allowed = self.radius_at(angle_of((point[0] - self.center[0],
                                           point[1] - self.center[1])))
        return gap < allowed - margin


def _split_arcs(samples, open_flag):
    """Group a circular ray sample into open and closed angular intervals."""
    n = len(samples)
    if n == 0:
        return [], []
    transitions = [i for i in range(n) if open_flag[i] != open_flag[(i - 1) % n]]
    if not transitions:
        full = (0.0, 2.0 * math.pi)
        return ([full], []) if open_flag[0] else ([], [full])
    arcs_open: list[tuple[float, float]] = []
    arcs_closed: list[tuple[float, float]] = []
    for k, start_i in enumerate(transitions):
        end_i = transitions[(k + 1) % len(transitions)]
        arc = (samples[start_i][0], samples[end_i][0])
        if open_flag[start_i]:
            arcs_open.append(arc)
        else:
            arcs_closed.append(arc)
    return arcs_open, arcs_closed


def _arc_sweep(start: float, end: float) -> float:
    sweep = (end - start) % (2.0 * math.pi)
    # A full turn is stored as a multiple of 2pi, which wraps to 0.
    if sweep <= 1e-8:
        return 2.0 * math.pi
    return sweep


def _arc_midpoint(center: Point, radius: float, start: float, end: float) -> Point:
    sweep = _arc_sweep(start, end)
    theta = start + 0.5 * sweep
    return (center[0] + radius * math.cos(theta),
            center[1] + radius * math.sin(theta))


def perceive(center: Point, radius: float, obstacles: list[Polygon],
             ray_count: int = 360) -> SightScan:
    """Cast rays around `center` and split the circle into open and closed arcs."""
    directions: list[float] = []
    step = 2.0 * math.pi / float(ray_count)
    directions.extend(i * step for i in range(ray_count))
    for vertex in polygon_vertices(obstacles):
        if dist(center, vertex) <= radius + 1e-6:
            directions.append(wrap_2pi(angle_of((vertex[0] - center[0],
                                                 vertex[1] - center[1]))))
    directions = sorted(set(round(theta, 6) for theta in directions))

    samples: list[tuple[float, float, Point]] = []
    for theta in directions:
        heading = (math.cos(theta), math.sin(theta))
        hit_distance, hit_point = first_obstacle_hit(center, heading, obstacles, radius)
        samples.append((theta, hit_distance, hit_point))

    # A wall that sits on the circle blocks that direction. Open means the
    # ray reaches the vision limit.
    open_flag = [sample[1] >= radius - 0.2 for sample in samples]
    arcs_open, arcs_closed = _split_arcs(samples, open_flag)

    open_points = []
    for start, end in arcs_open:
        sweep = _arc_sweep(start, end)
        # A sliver thinner than the ray step is a sampling gap, not an opening.
        if sweep < math.radians(2.0):
            continue
        theta = wrap_2pi(start + 0.5 * sweep)
        heading = (math.cos(theta), math.sin(theta))
        hit_distance, _ = first_obstacle_hit(center, heading, obstacles, radius)
        # The midpoint of an open arc lies on the vision circle. If this ray
        # meets a wall first, the arc was not actually open.
        if hit_distance < radius - 0.2:
            continue
        open_points.append((center[0] + radius * heading[0],
                            center[1] + radius * heading[1]))

    endpoints: list[Point] = []
    for vertex in polygon_vertices(obstacles):
        if visible_point(center, vertex, obstacles, radius):
            endpoints.append(vertex)
    # extreme hits of each closed sight: the boundary rays of the bundle
    for start, end in arcs_closed:
        for theta in (start, end):
            heading = (math.cos(theta), math.sin(theta))
            hit_distance, hit_point = first_obstacle_hit(center, heading, obstacles, radius)
            if hit_distance < radius - 1e-3 and all(dist(hit_point, old) > 0.35
                                                     for old in endpoints):
                endpoints.append(hit_point)

    return SightScan(center=center, radius=radius, samples=samples,
                     open_arcs=arcs_open, closed_arcs=arcs_closed,
                     open_points=open_points, bundle_endpoints=endpoints)


class ExploredSight:
    """Union of neighbor sights. Section 4.1: Se = union of Sn."""

    def __init__(self) -> None:
        self.scans: list[SightScan] = []

    def add(self, scan: SightScan) -> None:
        self.scans.append(scan)

    def contains_interior(self, point: Point, margin: float = 0.4) -> bool:
        return any(scan.contains_interior(point, margin) for scan in self.scans)


def wedge_polygon(scan: SightScan, start: float, end: float,
                  samples_on_arc: int = 18) -> list[Point]:
    """Polygon of one sight: the center plus the visible arc."""
    sweep = (end - start) % (2.0 * math.pi)
    if sweep <= 1e-9:
        sweep = 2.0 * math.pi
    points = [scan.center]
    steps = max(2, samples_on_arc)
    for i in range(steps + 1):
        theta = wrap_2pi(start + sweep * (i / steps))
        heading = (math.cos(theta), math.sin(theta))
        allowed = min(scan.radius, scan.radius_at(theta))
        points.append(add(scan.center, mul(unit(heading), allowed)))
    return points

import math
from dataclasses import dataclass, field



@dataclass
class Bundle:
    center: Point
    index: int
    rays: list[Segment] = field(default_factory=list)
    shortcut_rays: list[Segment] = field(default_factory=list)
    interior_angle: float = math.pi


def interior_angle(previous: Point, center: Point, nxt: Point) -> float:
    """Angle at `center` between the two neighbors. Straight is pi."""
    return angle_between(sub(previous, center), sub(nxt, center))


def _in_interior_wedge(previous: Point, center: Point, nxt: Point,
                       point: Point) -> bool:
    """True when `point` lies in the interior angle of the polyline."""
    toward_prev = sub(previous, center)
    toward_next = sub(nxt, center)
    toward_point = sub(point, center)
    if norm(toward_point) <= EPS:
        return False
    sweep = math.atan2(cross(toward_prev, toward_next), dot(toward_prev, toward_next))
    if abs(sweep) < 1e-3:
        return False
    to_point = math.atan2(cross(toward_prev, toward_point), dot(toward_prev, toward_point))
    if sweep > 0.0:
        return 1e-3 < to_point < sweep - 1e-3
    return sweep + 1e-3 < to_point < -1e-3


def build_bundle(center: Point, index: int, endpoints: list[Point],
                 previous: Point | None, nxt: Point | None) -> Bundle:
    rays = []
    for endpoint in endpoints:
        if dist(center, endpoint) <= EPS:
            continue
        rays.append((center, endpoint))
    rays.sort(key=lambda segment: math.atan2(segment[1][1] - center[1],
                                              segment[1][0] - center[0]))
    shortcut: list[Segment] = []
    angle = math.pi
    if previous is not None and nxt is not None:
        angle = interior_angle(previous, center, nxt)
        if angle < math.pi - 1e-3:
            shortcut = [ray for ray in rays
                        if _in_interior_wedge(previous, center, nxt, ray[1])]
    return Bundle(center=center, index=index, rays=rays,
                  shortcut_rays=shortcut, interior_angle=angle)


def build_sequence(centers: list[Point],
                   endpoints: list[list[Point]]) -> list[Bundle]:
    """One bundle per traveled center, ordered along the polyline."""
    bundles: list[Bundle] = []
    for index, center in enumerate(centers):
        previous = centers[index - 1] if index > 0 else None
        nxt = centers[index + 1] if index + 1 < len(centers) else None
        bundles.append(build_bundle(center, index, endpoints[index], previous, nxt))
    return bundles


def crossing_pairs(bundles: list[Bundle]) -> list[tuple[int, int, Point]]:
    """Interior intersections between rays that do not share a center."""
    hits: list[tuple[int, int, Point]] = []
    rays = [(bundle.index, ray) for bundle in bundles for ray in bundle.rays]
    for i in range(len(rays)):
        bi, (a, b) = rays[i]
        for j in range(i + 1, len(rays)):
            bj, (c, d) = rays[j]
            if bi == bj:
                continue
            if segments_properly_intersect(a, b, c, d):
                point = segment_intersection(a, b, c, d)
                if point is not None:
                    hits.append((bi, bj, point))
    return hits


def _shorten_before(segment: Segment, point: Point, gap: float) -> Segment:
    """Pull the far end back so the ray stops `gap` short of `point`."""
    origin, end = segment
    if dist(origin, point) <= gap + 1e-6:
        return (origin, origin)
    direction = unit(sub(point, origin))
    return (origin, add(point, mul(direction, -gap)))


def shorten_crossings(bundles: list[Bundle], gap: float = 0.35,
                      passes: int = 12) -> list[Bundle]:
    """Figure 9b -> 9c: shorten rays until distinct centers no longer cross.

    Section 3 also trims a length epsilon off the common vertex. That trim is
    applied by `trim_from_vertices`.
    """
    rays: list[tuple[int, Segment]] = [
        (bundle.index, ray) for bundle in bundles for ray in bundle.rays
    ]
    for _ in range(passes):
        changed = False
        for i in range(len(rays)):
            bi, left = rays[i]
            for j in range(i + 1, len(rays)):
                bj, right = rays[j]
                if bi == bj:
                    continue
                if not segments_properly_intersect(left[0], left[1], right[0], right[1]):
                    continue
                point = segment_intersection(left[0], left[1], right[0], right[1])
                if point is None:
                    continue
                rays[i] = (bi, _shorten_before(left, point, gap))
                rays[j] = (bj, _shorten_before(right, point, gap))
                left = rays[i][1]
                changed = True
        if not changed:
            break
    grouped: dict[int, list[Segment]] = {bundle.index: [] for bundle in bundles}
    for index, ray in rays:
        if dist(ray[0], ray[1]) > gap:
            grouped[index].append(ray)
    shortened: list[Bundle] = []
    for bundle in bundles:
        kept = grouped[bundle.index]
        shortcut = [ray for ray in kept if any(dist(ray[1], old[1]) < 0.8
                                               for old in bundle.shortcut_rays)]
        shortened.append(Bundle(center=bundle.center, index=bundle.index,
                                rays=kept, shortcut_rays=shortcut,
                                interior_angle=bundle.interior_angle))
    return shortened


def refilter_shortcuts(bundles: list[Bundle], centers: list[Point]) -> list[Bundle]:
    """Keep, on each interior bundle, the rays that sit in the interior angle."""
    filtered: list[Bundle] = []
    for bundle in bundles:
        previous = centers[bundle.index - 1] if bundle.index > 0 else None
        nxt = centers[bundle.index + 1] if bundle.index + 1 < len(centers) else None
        angle = math.pi
        shortcut: list[Segment] = []
        if previous is not None and nxt is not None:
            angle = interior_angle(previous, bundle.center, nxt)
            if angle < math.pi - 1e-3:
                shortcut = [ray for ray in bundle.rays
                            if _in_interior_wedge(previous, bundle.center, nxt, ray[1])]
        filtered.append(Bundle(center=bundle.center, index=bundle.index,
                               rays=bundle.rays, shortcut_rays=shortcut,
                               interior_angle=angle))
    return filtered


def fstar_segments(bundles: list[Bundle], epsilon: float) -> list[Segment]:
    """F* from Section 3: every shortcut ray, trimmed by epsilon at its vertex.

    Algorithm 1 puts one path vertex on each of these segments, in order
    along the polyline and around each bundle.
    """
    segments: list[Segment] = []
    for bundle in bundles:
        ordered = sorted(
            bundle.shortcut_rays,
            key=lambda ray: math.atan2(ray[1][1] - ray[0][1], ray[1][0] - ray[0][0]),
        )
        for origin, end in ordered:
            length = dist(origin, end)
            if length <= epsilon * 2.0:
                continue
            segments.append((lerp(origin, end, epsilon / length), end))
    return segments


def binding_segments(bundles: list[Bundle], centers: list[Point],
                     epsilon: float) -> list[Segment]:
    """One sliding segment per bent center: the shortcut ray the chord hits first.

    Algorithm 1 places a single path vertex on each segment of F*. Using the
    first ray crossed by the corner chord is the segment that actually
    constrains the shortcut. Free corners contribute no segment, so the path
    may cut them.
    """
    segments: list[Segment] = []
    for bundle in bundles:
        if bundle.index <= 0 or bundle.index >= len(centers) - 1:
            continue
        if not bundle.shortcut_rays:
            continue
        previous = centers[bundle.index - 1]
        nxt = centers[bundle.index + 1]
        chosen: Segment | None = None
        nearest = math.inf
        for ray in bundle.shortcut_rays:
            hit = segment_intersection(previous, nxt, ray[0], ray[1])
            if hit is None:
                continue
            gap = dist(ray[0], hit)
            if gap < nearest:
                nearest = gap
                chosen = ray
        if chosen is None:
            bisector = add(unit(sub(previous, bundle.center)),
                           unit(sub(nxt, bundle.center)))
            if norm(bisector) <= EPS:
                continue
            chosen = min(bundle.shortcut_rays,
                         key=lambda ray: _angle_to(bisector, sub(ray[1], ray[0])))
        length = dist(chosen[0], chosen[1])
        if length <= epsilon * 2.0:
            continue
        segments.append((lerp(chosen[0], chosen[1], epsilon / length), chosen[1]))
    return segments


def _angle_to(direction: Point, other: Point) -> float:
    return abs(math.atan2(cross(direction, other), dot(direction, other)))


def trim_from_vertices(bundles: list[Bundle], epsilon: float) -> list[Segment]:
    """Cut epsilon off every ray at its center. Section 3, construction of F*."""
    trimmed: list[Segment] = []
    for bundle in bundles:
        for origin, end in bundle.shortcut_rays:
            length = dist(origin, end)
            if length <= epsilon * 2.0:
                continue
            trimmed.append((lerp(origin, end, epsilon / length), end))
    return trimmed

def _optimal_on_segment(previous: Point, nxt: Point, segment: Segment) -> Point:
    """Point of `segment` minimizing |p-previous| + |p-nxt| (reflection)."""
    start, end = segment
    # Neighbors on opposite sides: the straight crossing is the minimum.
    crossing = segment_intersection(previous, nxt, start, end)
    if crossing is not None:
        return crossing
    direction = sub(end, start)
    length2 = dot(direction, direction)
    if length2 <= EPS:
        return start
    scale = dot(sub(nxt, start), direction) / length2
    projection = add(start, mul(direction, scale))
    reflected = sub(mul(projection, 2.0), nxt)
    chord = sub(reflected, previous)
    denominator = cross(chord, direction)
    if abs(denominator) <= EPS:
        at_start = dist(previous, start) + dist(start, nxt)
        at_end = dist(previous, end) + dist(end, nxt)
        return start if at_start <= at_end else end
    along = cross(sub(start, previous), direction) / denominator
    point = add(previous, mul(chord, along))
    parameter = dot(sub(point, start), direction) / length2
    return lerp(start, end, max(0.0, min(1.0, parameter)))


def _project_parameter(point: Point, segment: Segment) -> Point:
    start, end = segment
    direction = sub(end, start)
    length2 = dot(direction, direction)
    if length2 <= EPS:
        return start
    parameter = dot(sub(point, start), direction) / length2
    return lerp(start, end, max(0.0, min(1.0, parameter)))


def dap(origin: Point, goal: Point, segments: list[Segment],
        obstacles: list[Polygon], delta: float = 1e-8,
        iterations: int = 80) -> list[Point]:
    """Shorten `origin → goal` with one path vertex on each segment of F*."""
    if not segments:
        return [origin, goal]
    points: list[Point] = [origin]
    for segment in segments:
        # Start next to the bundle vertex (the trimmed end nearer the center
        # is segment[0]) so the initial polyline follows the traveled path.
        points.append(segment[0])
    points.append(goal)

    previous_length = path_length(points)
    for _ in range(iterations):
        for index, segment in enumerate(segments, start=1):
            candidate = _optimal_on_segment(points[index - 1], points[index + 1], segment)
            # Keep the candidate on the segment even if reflection drifted.
            candidate = _project_parameter(candidate, segment)
            if segment_hits_obstacles(points[index - 1], candidate, obstacles):
                continue
            if segment_hits_obstacles(candidate, points[index + 1], obstacles):
                continue
            before = dist(points[index - 1], points[index]) + dist(points[index], points[index + 1])
            after = dist(points[index - 1], candidate) + dist(candidate, points[index + 1])
            if after <= before + 1e-12:
                points[index] = candidate
        length = path_length(points)
        if abs(previous_length - length) < delta:
            break
        previous_length = length
    return _drop_repeats(points)


def shorten_free_corners(points: list[Point], obstacles: list[Polygon],
                         delta: float = 1e-4) -> list[Point]:
    """Paper Section 3.1: while the angle at a vertex is < pi and the chord
    is free, drop that vertex. Used when a bundle has no ray inside the
    corner, so there is no segment left to slide on.
    """
    path = list(points)
    changed = True
    guard = 0
    while changed and guard < len(points) + 2:
        changed = False
        guard += 1
        index = 1
        while index < len(path) - 1:
            if segment_hits_obstacles(path[index - 1], path[index + 1], obstacles):
                index += 1
                continue
            path.pop(index)
            changed = True
        if path_length(path) + delta >= path_length(points) and not changed:
            break
    return path


def _drop_repeats(points: list[Point]) -> list[Point]:
    cleaned = [points[0]]
    for point in points[1:]:
        if dist(cleaned[-1], point) > 0.8:
            cleaned.append(point)
    return cleaned

START: Point = (30.0, 22.0)
GOAL: Point = (86.0, 86.0)
RADIUS: float = 16.0


def obstacles() -> list[Polygon]:
    return [
        rect(0, 0, 100, 4),
        rect(0, 0, 4, 100),
        rect(96, 0, 100, 100),
        rect(0, 96, 100, 100),
        # three blocks inside the disk at the start
        rect(16, 14, 24, 24),
        rect(40, 12, 50, 22),
        rect(18, 28, 26, 36),
        # north leg x = 28..40. Right wall stops at y = 52.
        rect(16, 32, 28, 72),
        rect(40, 28, 50, 52),
        # east leg y = 56..72. Floor starts at x = 52, so the elbow is open.
        rect(16, 72, 78, 80),
        rect(52, 44, 78, 56),
        rect(70, 56, 78, 72),
        # passage south of the start, then east and north to the goal
        rect(48, 4, 90, 14),
        rect(84, 40, 92, 58),
        rect(78, 74, 96, 92),
        rect(4, 70, 18, 90),
    ]

from dataclasses import dataclass, field



@dataclass
class OpenPoint:
    point: Point
    origin: Point
    origin_index: int
    active: bool = True


@dataclass
class PointMark:
    """One open point as seen from a pose, for the study figure."""

    point: Point
    origin_index: int
    distance: float
    angle: float
    score: float
    active: bool
    chosen: bool
    status: str


@dataclass
class Snapshot:
    """One robot stop: the sight, the local star, and the pose."""

    center: Point
    scan: SightScan
    open_points: list[Point]
    edges: list[Segment]
    marks: list[PointMark] = field(default_factory=list)


@dataclass
class Escape:
    entry: list[Point]
    bundles_raw: list[Bundle]
    bundles: list[Bundle]
    crossings: list[tuple[int, int, Point]]
    exit_path: list[Point]
    entry_length: float
    exit_length: float
    return_point: Point


@dataclass
class MissionResult:
    start: Point
    goal: Point
    radius: float
    obstacles: list[Polygon]
    trajectory: list[Point]
    snapshots: list[Snapshot]
    escape: Escape | None
    reached_goal: bool
    focus_count: int = 0
    log: list[str] = field(default_factory=list)
    alpha: float = 1.0
    beta: float = 1.0


def _in_neighbor_sight(scan: SightScan, point: Point, obstacles: list[Polygon]) -> bool:
    if dist(scan.center, point) > scan.radius + 1e-6:
        return False
    return not segment_hits_obstacles(scan.center, point, obstacles)


def _marks(scan: SightScan, current: Point, goal: Point, alpha: float, beta: float,
          explored: ExploredSight, fresh: list[Point],
          origin_index: int) -> list[PointMark]:
    fresh_keys = {(round(point[0], 3), round(point[1], 3)) for point in fresh}
    marks: list[PointMark] = []
    for point in scan.open_points:
        distance, angle, score = ranking_parts(point, current, goal, alpha, beta)
        key = (round(point[0], 3), round(point[1], 3))
        if explored.contains_interior(point):
            status, active = "trong vùng đã đi", False
        elif key not in fresh_keys:
            status, active = "trùng điểm cũ", False
        else:
            status, active = "đang mở", True
        marks.append(PointMark(point, origin_index, distance, angle, score,
                               active, False, status))
    return marks


def _tag_choice(snapshot: Snapshot, chosen: Point, origin_index: int,
                goal: Point, alpha: float, beta: float, local: bool) -> None:
    for mark in snapshot.marks:
        if dist(mark.point, chosen) < 1.0:
            mark.chosen = True
            mark.status = "được chọn"
            return
    if local:
        return
    distance, angle, score = ranking_parts(chosen, snapshot.center, goal, alpha, beta)
    snapshot.marks.append(PointMark(
        chosen, origin_index, distance, angle, score, True, True, "quay về"))


# A path that merely misses the wall polygon still looks like a hit when it
# passes closer than this. The paper's exit stays inside the free channel.
CLEARANCE = 1.5


def _path_clear(points: list[Point], obstacles: list[Polygon]) -> bool:
    for index in range(1, len(points)):
        start, end = points[index - 1], points[index]
        if segment_hits_obstacles(start, end, obstacles):
            return False
        if segment_clearance(start, end, obstacles) < CLEARANCE:
            return False
    return True


def _escape(centers: list[Point], endpoint_lists: list[list[Point]],
            obstacles: list[Polygon], return_point: Point,
            epsilon: float) -> Escape:
    """Bundles along `centers` (dead-end first) and a DAP exit to `return_point`."""
    raw = build_sequence(centers, endpoint_lists)
    crossings = crossing_pairs(raw)
    shortened = refilter_shortcuts(shorten_crossings(raw), centers)
    # Algorithm 1 runs on the whole trimmed sequence F*, not on one ray.
    segments = fstar_segments(shortened, epsilon)
    # The arc midpoint can sit in a corner. Stand at the last point of that
    # ray which still clears the walls; both paths below use this same pose.
    pose = pull_clear(centers[-1], return_point, obstacles, CLEARANCE)
    retrace = list(centers)
    if dist(retrace[-1], pose) > 1e-6:
        retrace.append(pose)
    exit_path = dap(centers[0], pose, segments, obstacles)
    if not _path_clear(exit_path, obstacles) or path_length(exit_path) > path_length(retrace) + 1e-6:
        fallback = shorten_free_corners(retrace, obstacles)
        exit_path = fallback if _path_clear(fallback, obstacles) else retrace
    entry = list(reversed(centers))
    return Escape(
        entry=entry,
        bundles_raw=raw,
        bundles=shortened,
        crossings=crossings,
        exit_path=exit_path,
        entry_length=path_length(retrace),
        exit_length=path_length(exit_path),
        return_point=pose,
    )


def run_mission(start: Point, goal: Point, radius: float,
                obstacles: list[Polygon], alpha: float = 1.0, beta: float = 1.0,
                max_steps: int = 24, goal_tolerance: float = 1.0,
                epsilon: float = 0.25) -> MissionResult:
    current = start
    explored = ExploredSight()
    memory: list[OpenPoint] = []
    trajectory: list[Point] = [start]
    snapshots: list[Snapshot] = []
    centers: list[Point] = [start]
    endpoint_lists: list[list[Point]] = []
    escape: Escape | None = None
    focus_count = 0
    log: list[str] = []
    reached = False

    for step in range(max_steps):
        scan = perceive(current, radius, obstacles)
        explored.add(scan)
        endpoint_lists.append(list(scan.bundle_endpoints))

        for record in memory:
            if record.active and explored.contains_interior(record.point):
                record.active = False

        fresh: list[Point] = []
        for point in scan.open_points:
            if explored.contains_interior(point):
                continue
            if any(dist(point, record.point) < 2.0 for record in memory):
                continue
            memory.append(OpenPoint(point, current, len(centers) - 1, True))
            fresh.append(point)

        ranked: list[tuple[float, OpenPoint]] = []
        for record in memory:
            if not record.active:
                continue
            score = ranking_score(record.point, current, goal, alpha, beta)
            ranked.append((score, record))
        ranked.sort(key=lambda item: (-item[0], dist(item[1].point, goal)))

        edges = [(current, record.point) for record in memory
                 if record.origin_index == len(centers) - 1 and record.active]
        marks = _marks(scan, current, goal, alpha, beta, explored, fresh,
                       len(centers) - 1)
        snapshots.append(Snapshot(current, scan, fresh, edges, marks))
        active_txt = ", ".join(
            f"({record.point[0]:.0f},{record.point[1]:.0f})@C{record.origin_index}"
            for record in memory if record.active)
        log.append(
            f"C{len(centers) - 1} ({current[0]:.1f}, {current[1]:.1f}) "
            f"open={len(scan.open_points)} active=[{active_txt}]")

        if dist(current, goal) <= goal_tolerance:
            reached = True
            log.append("reached goal")
            break
        if dist(current, goal) <= radius and not segment_hits_obstacles(current, goal, obstacles):
            trajectory.append(goal)
            reached = True
            log.append("goal inside the vision disk, straight step")
            break
        if not ranked:
            log.append("no active open point")
            break

        score, chosen = ranked[0]
        target = chosen.point
        local = _in_neighbor_sight(scan, target, obstacles)
        if not local:
            # The globally best point can sit in a corner of the same alley.
            # The way out is the best open point left at the oldest junction.
            outside = [
                item for item in ranked
                if not _in_neighbor_sight(scan, item[1].point, obstacles)
            ]
            oldest = min(item[1].origin_index for item in outside)
            pool = [item for item in outside if item[1].origin_index == oldest]
            score, chosen = max(pool, key=lambda item: item[0])
            target = chosen.point
        _tag_choice(snapshots[-1], target, chosen.origin_index, goal, alpha, beta, local)
        log.append(
            f"  next ({target[0]:.1f}, {target[1]:.1f}) "
            f"score={score:.3f} from C{chosen.origin_index} "
            f"{'straight' if local else 'DAP'}")

        if local:
            if segment_hits_obstacles(current, target, obstacles):
                chosen.active = False
                log.append("  rejected, segment hits an obstacle")
                continue
            trajectory.append(target)
            current = target
            centers.append(current)
            chosen.active = False
            continue

        # Blind alley: the best open point is outside the current sight.
        # The sequence F is the bundles at the centers from here back to
        # the pose that first saw the return point.
        chain = list(reversed(centers[chosen.origin_index:]))
        chain_endpoints = list(reversed(endpoint_lists[chosen.origin_index:]))
        # The chain must include the current center, which is centers[-1],
        # and its endpoints were stored at the start of this iteration.
        this_escape = _escape(chain, chain_endpoints, obstacles, target, epsilon)
        trajectory.extend(this_escape.exit_path[1:])
        log.append(
            f"  escape polyline {this_escape.entry_length:.2f} "
            f"dap {this_escape.exit_length:.2f} "
            f"crossings {len(this_escape.crossings)} "
            f"centers {len(chain)}")
        # Figure 8 and Figure 9 show the first real alley. Algorithm 2
        # itself keeps going until the goal.
        if escape is None and len(chain) >= 3:
            escape = this_escape
            focus_count = len(snapshots)
            break
        current = target
        centers.append(current)
        chosen.active = False

    return MissionResult(
        start=start, goal=goal, radius=radius, obstacles=obstacles,
        trajectory=trajectory, snapshots=snapshots, escape=escape,
        reached_goal=reached, focus_count=focus_count or len(snapshots),
        log=log, alpha=alpha, beta=beta,
    )

def _load_plotting_backend() -> None:
    """Keep the geometry usable by the ROS adapter without Matplotlib."""
    global plt, Circle, MplPolygon
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon as MplPolygon



def _walls(ax, result: MissionResult) -> None:
    for polygon in result.obstacles:
        ax.add_patch(MplPolygon(
            polygon, closed=True, facecolor="#d5d5d5", edgecolor="#4a4a4a",
            hatch="///", linewidth=0.6, zorder=2))


def _vision(ax, center: Point, radius: float) -> None:
    ax.add_patch(Circle(
        center, radius, facecolor="#cfe8c9", edgecolor="none",
        alpha=0.28, zorder=1))
    ax.add_patch(Circle(
        center, radius, facecolor="none", edgecolor="#d62728",
        linewidth=1.0, linestyle=(0, (1.2, 1.6)), zorder=3))


def _sights(ax, snapshot: Snapshot) -> None:
    scan = snapshot.scan
    for start, end in scan.closed_arcs:
        ax.add_patch(MplPolygon(
            wedge_polygon(scan, start, end, 16), closed=True,
            facecolor="#f3b6c4", edgecolor="none", alpha=0.55, zorder=1))
    for start, end in scan.open_arcs:
        ax.add_patch(MplPolygon(
            wedge_polygon(scan, start, end, 18), closed=True,
            facecolor="#8fd18f", edgecolor="none", alpha=0.40, zorder=1))


def _shown(result: MissionResult) -> list:
    count = result.focus_count or len(result.snapshots)
    return result.snapshots[:count]


def _entry(result: MissionResult) -> list[Point]:
    return [snapshot.center for snapshot in _shown(result)]


def _fans(ax, result: MissionResult, link_inactive: bool) -> None:
    """Rays from each pose to the open points of that pose."""
    labeled_active = False
    labeled_idle = False
    for snapshot in _shown(result):
        center = snapshot.center
        for mark in snapshot.marks:
            if mark.status == "quay về":
                continue
            idle = mark.status == "trong vùng đã đi"
            if idle and not link_inactive:
                continue
            color = "#9a9a9a" if idle else "#333333"
            width = 0.6 if idle else 0.9
            ax.plot([center[0], mark.point[0]], [center[1], mark.point[1]],
                    color=color, linewidth=width, linestyle="--", zorder=4,
                    label=("open point đã thấy" if idle and not labeled_idle
                           else "open point" if not idle and not labeled_active
                           else None))
            if idle:
                labeled_idle = True
                ax.scatter([mark.point[0]], [mark.point[1]], s=18,
                           facecolor="none", edgecolor="#666666",
                           linewidth=0.8, zorder=5)
            elif mark.chosen:
                ax.scatter([mark.point[0]], [mark.point[1]], s=28,
                           c="#d62728", zorder=6)
            else:
                labeled_active = True
                ax.scatter([mark.point[0]], [mark.point[1]], s=16,
                           c="#1f4e79", zorder=5)


def _pose_label(result: MissionResult, point: Point, fallback: int) -> str:
    for index, snapshot in enumerate(result.snapshots):
        if dist(snapshot.center, point) < 1e-4:
            return f"C{index}"
    return f"C{fallback}"


def _poses(ax, result: MissionResult, centers: list[Point]) -> None:
    for index, point in enumerate(centers):
        ax.scatter([point[0]], [point[1]], s=22, c="#1f4e79", zorder=6)
        ax.annotate(_pose_label(result, point, index), point,
                    textcoords="offset points", xytext=(3, 3),
                    fontsize=8, color="#1f4e79", zorder=7)


def _path(ax, points: list[Point], color: str, width: float, style: str,
          label: str) -> None:
    if len(points) < 2:
        return
    ax.plot([point[0] for point in points], [point[1] for point in points],
            color=color, linewidth=width, linestyle=style,
            solid_capstyle="round", zorder=5, label=label)


def _goal(ax, goal: Point) -> None:
    ax.scatter([goal[0]], [goal[1]], marker="*", s=160, c="#d62728",
               zorder=6, label="goal")


def _frame(ax, title: str) -> None:
    ax.set_title(title, fontsize=10)
    ax.set_xlim(-2, 102)
    ax.set_ylim(-2, 102)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, color="#eeeeee", linewidth=0.4)
    ax.tick_params(labelsize=7)
    ax.legend(loc="upper left", fontsize=7, framealpha=0.92)


def range_error(result: MissionResult) -> float:
    """Largest |distance(Ci, open point) - r| over the mission."""
    worst = 0.0
    for snapshot in _shown(result):
        for mark in snapshot.marks:
            if mark.status == "quay về":
                continue
            worst = max(worst, abs(dist(snapshot.center, mark.point) - result.radius))
    return worst


def render(result: MissionResult, path: str) -> None:
    _load_plotting_backend()
    entry = _entry(result)
    error = range_error(result)
    figure, axes = plt.subplots(2, 2, figsize=(12.5, 11.5))
    figure.suptitle(
        f"r = {result.radius:.0f}    open point nằm trên vòng tròn    "
        f"sai số lớn nhất {error:.4f}",
        fontsize=12)

    # Figure 8a — sights, vision disks, entry
    ax = axes[0][0]
    for snapshot in _shown(result):
        _vision(ax, snapshot.center, result.radius)
        _sights(ax, snapshot)
    _walls(ax, result)
    _path(ax, entry, "#d62728", 2.0, "-", "đường đi vào")
    _fans(ax, result, link_inactive=False)
    _poses(ax, result, entry)
    _goal(ax, result.goal)
    _frame(ax, "Hình 8a  —  tầm nhìn, closed/open sight, open point hợp lệ")

    # Figure 8b — visibility graph, every valid open point
    ax = axes[0][1]
    for snapshot in _shown(result):
        _vision(ax, snapshot.center, result.radius)
    _walls(ax, result)
    _path(ax, entry, "#d62728", 1.8, "-", "đường đi vào")
    _fans(ax, result, link_inactive=True)
    _poses(ax, result, entry)
    _goal(ax, result.goal)
    _frame(ax, "Hình 8b  —  chùm tia tới toàn bộ open point của mỗi Ci")

    # Figure 9b — raw bundles along the alley
    ax = axes[1][0]
    for center in (result.escape.entry if result.escape else entry):
        _vision(ax, center, result.radius)
    _walls(ax, result)
    if result.escape is not None:
        first = True
        for bundle in result.escape.bundles_raw:
            for origin, end in bundle.rays:
                ax.plot([origin[0], end[0]], [origin[1], end[1]],
                        color="#1f77b4", linewidth=0.7, zorder=4,
                        label="chùm tia" if first else None)
                first = False
        for _, _, point in result.escape.crossings:
            ax.scatter([point[0]], [point[1]], s=14, c="#d62728", zorder=6)
    _path(ax, result.escape.entry if result.escape else entry,
          "#444444", 1.4, "--", "đường đi vào")
    _poses(ax, result, result.escape.entry if result.escape else entry)
    _goal(ax, result.goal)
    crossings = 0 if result.escape is None else len(result.escape.crossings)
    _frame(ax, f"Hình 9b  —  chùm tia, giao nhau ({crossings})")

    # Figure 9c — exit
    ax = axes[1][1]
    centers = result.escape.entry if result.escape else entry
    for center in centers:
        _vision(ax, center, result.radius)
    _walls(ax, result)
    if result.escape is not None:
        first = True
        for bundle in result.escape.bundles:
            for origin, end in bundle.rays:
                ax.plot([origin[0], end[0]], [origin[1], end[1]],
                        color="#1f77b4", linewidth=0.7, zorder=4,
                        label="chùm tia đã rút" if first else None)
                first = False
        long_way = list(reversed(result.escape.entry))
        if dist(long_way[-1], result.escape.return_point) > 1e-3:
            long_way.append(result.escape.return_point)
        _path(ax, long_way, "#444444", 1.2, "--", "đường đi vào")
        _path(ax, result.escape.exit_path, "#d62728", 2.2, "-", "đường đi ra")
        xs = [point[0] for point in result.escape.exit_path]
        ys = [point[1] for point in result.escape.exit_path]
        ax.scatter(xs, ys, s=18, c="#d62728", zorder=6)
        ax.text(6, 8,
                f"L vào {result.escape.entry_length:.1f}\n"
                f"L ra {result.escape.exit_length:.1f}",
                fontsize=8, family="monospace",
                bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.9})
    _poses(ax, result, centers)
    _goal(ax, result.goal)
    _frame(ax, "Hình 9c  —  tầm nhìn, chùm tia, đường đi ra")

    figure.tight_layout(rect=(0, 0, 1, 0.96))
    figure.savefig(path, dpi=140)
    plt.close(figure)

from pathlib import Path



def main() -> None:
    result = run_mission(START, GOAL, RADIUS, obstacles())
    out_dir = Path(__file__).resolve().parent / "output"
    out_dir.mkdir(exist_ok=True)
    figure_path = out_dir / "fig8_fig9.png"
    render(result, str(figure_path))
    print(f"steps {len(result.snapshots)}  reached {result.reached_goal}")
    print(f"trajectory length { _length(result.trajectory):.2f}")
    if result.escape is None:
        print("no blind-alley escape")
    else:
        print(f"entry {result.escape.entry_length:.2f}  "
              f"exit {result.escape.exit_length:.2f}  "
              f"crossings {len(result.escape.crossings)}")
    print(f"figure {figure_path}")
    for line in result.log:
        print(line)


def _length(points) -> float:
    return path_length(points)



if __name__ == "__main__":
    main()
