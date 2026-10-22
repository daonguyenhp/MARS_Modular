"""Incremental form of the Figure 8/9 planner for a moving 2-D robot.

Only ``plan_next`` advances the planner. A simulator must finish executing the
returned collision-free polyline before asking for another decision. The scan,
open-point ranking, bundle construction and DAP all use fig8_fig9.py.
"""

from dataclasses import dataclass
import math

import fig8_fig9 as paper


def raycast_ranges(pose, yaw, radius, obstacles, ray_count=180):
    """One full LiDAR sweep from the robot's *current* pose.

    Returns infinity for directions with no wall inside sensing range, as
    required by sensor_msgs/LaserScan.
    """
    readings = []
    for index in range(ray_count):
        angle = yaw - math.pi + index * (2.0 * math.pi / ray_count)
        distance, _ = paper.first_obstacle_hit(
            pose, (math.cos(angle), math.sin(angle)), obstacles, radius)
        readings.append(distance if distance < radius - 1e-5 else math.inf)
    return readings


@dataclass
class Plan:
    event: str
    path: list[paper.Point]
    scan: paper.SightScan
    escape: paper.Escape | None = None


class OnlineMission:
    def __init__(self, start, goal, radius, obstacles, max_decisions=80):
        if any(paper.point_in_polygon(start, o) for o in obstacles):
            raise ValueError("start lies inside an obstacle")
        if any(paper.point_in_polygon(goal, o) for o in obstacles):
            raise ValueError("goal lies inside an obstacle")
        self.start, self.goal, self.radius = start, goal, radius
        self.obstacles = obstacles
        self.current = start
        self.max_decisions = max_decisions
        self.explored = paper.ExploredSight()
        self.memory: list[paper.OpenPoint] = []
        self.centers = [start]
        self.endpoint_lists: list[list[paper.Point]] = []
        self.snapshots: list[paper.Snapshot] = []
        self.escapes: list[paper.Escape] = []
        self.done = False
        self.stop_reason = ""

    def plan_next(self) -> Plan:
        if self.done:
            raise RuntimeError("mission already ended")
        if len(self.snapshots) >= self.max_decisions:
            self.done, self.stop_reason = True, "max_decisions"
            raise RuntimeError(self.stop_reason)

        current = self.current
        scan = paper.perceive(current, self.radius, self.obstacles)
        self.explored.add(scan)
        self.endpoint_lists.append(list(scan.bundle_endpoints))
        for record in self.memory:
            if record.active and self.explored.contains_interior(record.point):
                record.active = False

        fresh = []
        for xy in scan.open_points:
            if self.explored.contains_interior(xy):
                continue
            if any(paper.dist(xy, rec.point) < 2.0 for rec in self.memory):
                continue
            self.memory.append(paper.OpenPoint(
                xy, current, len(self.centers) - 1, True))
            fresh.append(xy)

        ranked = [(paper.ranking_score(rec.point, current, self.goal), rec)
                  for rec in self.memory if rec.active]
        ranked.sort(key=lambda item: (-item[0],
                                      paper.dist(item[1].point, self.goal)))
        edges = [(current, rec.point) for rec in self.memory
                 if rec.origin_index == len(self.centers) - 1 and rec.active]
        marks = paper._marks(scan, current, self.goal, 1.0, 1.0,
                             self.explored, fresh, len(self.centers) - 1)
        self.snapshots.append(paper.Snapshot(current, scan, fresh, edges, marks))

        if paper.dist(current, self.goal) <= 1.0:
            self.done, self.stop_reason = True, "reached_goal"
            return Plan("reached_goal", [current], scan)
        if (paper.dist(current, self.goal) <= self.radius and
                not paper.segment_hits_obstacles(
                    current, self.goal, self.obstacles)):
            self.current = self.goal
            self.centers.append(self.goal)
            return Plan("go_to_goal", [current, self.goal], scan)
        if not ranked:
            self.done, self.stop_reason = True, "no_active_open_point"
            return Plan("no_active_open_point", [current], scan)

        chosen = ranked[0][1]
        local = paper._in_neighbor_sight(scan, chosen.point, self.obstacles)
        if not local:
            outside = [rec for _, rec in ranked if not paper._in_neighbor_sight(
                scan, rec.point, self.obstacles)]
            oldest = min(rec.origin_index for rec in outside)
            pool = [(score, rec) for score, rec in ranked
                    if rec.origin_index == oldest and rec in outside]
            chosen = max(pool, key=lambda item: item[0])[1]
        paper._tag_choice(self.snapshots[-1], chosen.point,
                          chosen.origin_index, self.goal, 1.0, 1.0, local)

        if local:
            path = [current, chosen.point]
            if paper.segment_hits_obstacles(*path, self.obstacles):
                self.done, self.stop_reason = True, "invalid_local_edge"
                return Plan("invalid_local_edge", [current], scan)
            event, escape = "open_point", None
        else:
            chain = list(reversed(self.centers[chosen.origin_index:]))
            endpoints = list(reversed(
                self.endpoint_lists[chosen.origin_index:]))
            escape = paper._escape(chain, endpoints, self.obstacles,
                                   chosen.point, epsilon=0.25)
            path = escape.exit_path
            self.escapes.append(escape)
            event = "dap_escape"

        chosen.active = False
        # In the paper figure the arc midpoint is pulled to a free return pose.
        # A physical robot must plan its next step from that *actual* endpoint.
        self.current = path[-1]
        self.centers.append(self.current)
        return Plan(event, path, scan, escape)
