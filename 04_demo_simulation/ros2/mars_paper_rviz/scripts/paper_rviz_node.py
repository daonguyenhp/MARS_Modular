#!/usr/bin/env python3
"""Publish the existing fig8_fig9.py simulation as an RViz timeline.

This is a display adapter: sight, ranking, bundles and DAP are calculated by
fig8_fig9.py, not reconstructed from hand-written RViz coordinates.
"""

import math
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


CORE_DIR = Path(get_package_share_directory("mars_paper_rviz")) / "core"
sys.path.insert(0, str(CORE_DIR))
import fig8_fig9 as paper  # noqa: E402 - installed paper code, not a duplicate


def point(xy, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(xy[0]), float(xy[1]), float(z)
    return p


def marker(stamp, namespace, marker_id, kind, color, scale=0.25):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = stamp
    m.ns = namespace
    m.id = marker_id
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = float(scale)
    m.scale.y = float(scale)
    m.scale.z = float(scale)
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def line(stamp, namespace, points, color, width=0.35):
    m = marker(stamp, namespace, 0, Marker.LINE_STRIP, color, width)
    m.points = [point(p, 0.4) for p in points]
    return m


def spheres(stamp, namespace, points, color, diameter=1.0):
    m = marker(stamp, namespace, 0, Marker.SPHERE_LIST, color, diameter)
    m.points = [point(p, 0.6) for p in points]
    return m


def ros_path(stamp, positions):
    msg = RosPath()
    msg.header.frame_id = "map"
    msg.header.stamp = stamp
    for xy in positions:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position = point(xy, 0.5)
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


class PaperRvizNode(Node):
    def __init__(self):
        super().__init__("paper_rviz")
        self.declare_parameter("playback_hz", 1.5)
        self.result = paper.run_mission(
            paper.START, paper.GOAL, paper.RADIUS, paper.obstacles())
        self.goal_invalid = any(
            paper.point_in_polygon(self.result.goal, obstacle)
            for obstacle in self.result.obstacles)
        if self.goal_invalid:
            self.get_logger().warn(
                "Original Figure 8/9 GOAL lies inside an obstacle. "
                "Playback demonstrates the first blind-alley escape, not arrival at goal.")
        if self.result.escape:
            escape = self.result.escape
            self.get_logger().info(
                f"Paper run: {len(self.result.snapshots)} scans, "
                f"{len(escape.crossings)} raw crossings, "
                f"entry/retrace {escape.entry_length:.2f}, "
                f"DAP exit {escape.exit_length:.2f}")
        else:
            self.get_logger().warn("Paper run has no blind-alley escape")

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.map_pub = self.create_publisher(OccupancyGrid, "/paper/map", latched)
        self.pose_pub = self.create_publisher(PoseStamped, "/paper/robot_pose", 10)
        self.markers_pub = self.create_publisher(
            MarkerArray, "/paper/markers", 10)
        self.entry_pub = self.create_publisher(
            RosPath, "/paper/travelled_entry", 10)
        self.baseline_pub = self.create_publisher(
            RosPath, "/paper/retrace_baseline", 10)
        self.exit_pub = self.create_publisher(RosPath, "/paper/exit_path", 10)
        self.status_pub = self.create_publisher(String, "/paper/status", latched)

        self.timeline = [("entry", i, s.center)
                         for i, s in enumerate(self.result.snapshots)]
        if self.result.escape:
            dead_end = self.result.escape.exit_path[0]
            self.timeline += [("raw_bundles", len(self.result.snapshots) - 1,
                               dead_end),
                              ("trimmed_bundles", len(self.result.snapshots) - 1,
                               dead_end)]
            self.timeline += [("exit", i, xy)
                              for i, xy in enumerate(
                                  self.result.escape.exit_path[1:], 1)]
        self.cursor = 0
        self.map_published = False
        hz = max(0.2, float(self.get_parameter("playback_hz").value))
        self.timer = self.create_timer(1.0 / hz, self.tick)

    def publish_map(self, stamp):
        # This is the ground-truth scenario map, not the robot's explored map.
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = stamp
        grid.info.resolution = 1.0
        grid.info.width = 100
        grid.info.height = 100
        grid.info.origin.orientation.w = 1.0
        grid.data = [
            100 if any(paper.point_in_polygon((x + 0.5, y + 0.5), obstacle)
                       for obstacle in self.result.obstacles) else 0
            for y in range(100) for x in range(100)
        ]
        self.map_pub.publish(grid)

    def add_sights(self, markers, stamp, last_index):
        open_triangles = marker(
            stamp, "open_sight", 0, Marker.TRIANGLE_LIST,
            (0.2, 0.85, 0.25, 0.22), 1.0)
        closed_triangles = marker(
            stamp, "closed_sight", 0, Marker.TRIANGLE_LIST,
            (1.0, 0.35, 0.45, 0.25), 1.0)
        for snapshot in self.result.snapshots[:last_index + 1]:
            for spans, dest in ((snapshot.scan.open_arcs, open_triangles),
                                (snapshot.scan.closed_arcs, closed_triangles)):
                for start, end in spans:
                    wedge = paper.wedge_polygon(snapshot.scan, start, end)
                    for i in range(1, len(wedge) - 1):
                        dest.points.extend((point(wedge[0], 0.04),
                                            point(wedge[i], 0.04),
                                            point(wedge[i + 1], 0.04)))
        markers.extend((open_triangles, closed_triangles))

    def add_sight_graph(self, markers, stamp, last_index):
        rays = marker(stamp, "open_point_rays", 0, Marker.LINE_LIST,
                      (0.35, 0.65, 1.0, 0.75), 0.16)
        active, inactive, chosen = [], [], []
        for snapshot in self.result.snapshots[:last_index + 1]:
            for mark in snapshot.marks:
                if mark.status == "quay về":
                    continue
                rays.points.extend((point(snapshot.center, 0.2),
                                    point(mark.point, 0.2)))
                if mark.chosen:
                    chosen.append(mark.point)
                elif mark.active:
                    active.append(mark.point)
                else:
                    inactive.append(mark.point)
        markers.append(rays)
        markers.append(spheres(stamp, "open_active", active,
                               (0.1, 0.35, 0.95, 1.0), 1.1))
        markers.append(spheres(stamp, "open_inactive", inactive,
                               (0.5, 0.5, 0.5, 0.8), 0.85))
        markers.append(spheres(stamp, "open_chosen", chosen,
                               (1.0, 0.12, 0.12, 1.0), 1.4))

    def add_bundles(self, markers, stamp, phase):
        escape = self.result.escape
        if escape is None or phase == "entry":
            return
        raw = phase == "raw_bundles"
        bundles = escape.bundles_raw if raw else escape.bundles
        rays = marker(stamp, "bundle_rays", 0, Marker.LINE_LIST,
                      (0.0, 0.65, 0.9, 0.9), 0.22)
        for bundle in bundles:
            for start, end in bundle.rays:
                rays.points.extend((point(start, 0.5), point(end, 0.5)))
        markers.append(rays)
        markers.append(spheres(stamp, "bundle_centers",
                               [b.center for b in bundles],
                               (1.0, 0.55, 0.0, 1.0), 1.3))
        if raw:
            markers.append(spheres(stamp, "raw_crossings",
                                   [hit[2] for hit in escape.crossings],
                                   (1.0, 0.05, 0.05, 1.0), 0.9))

    def publish_markers(self, stamp, phase, scan_index, pose_xy):
        array = MarkerArray()
        clear = marker(stamp, "clear", 0, Marker.DELETEALL,
                       (0.0, 0.0, 0.0, 0.0))
        clear.action = Marker.DELETEALL
        array.markers.append(clear)
        goal = marker(stamp, "goal_reference", 0, Marker.SPHERE,
                      (1.0, 0.1, 0.1, 0.9), 2.0)
        goal.pose.position = point(self.result.goal, 1.0)
        array.markers.append(goal)
        if self.goal_invalid:
            warning = marker(stamp, "invalid_goal_label", 0,
                             Marker.TEXT_VIEW_FACING,
                             (1.0, 0.25, 0.1, 1.0), 2.6)
            warning.pose.position = point((80, 93), 2.0)
            warning.text = "GOAL IN OBSTACLE (source scenario)"
            array.markers.append(warning)

        if scan_index >= 0:
            self.add_sights(array.markers, stamp, scan_index)
            self.add_sight_graph(array.markers, stamp, scan_index)
            # The Python mission does not rescan during the DAP return.
            if phase != "exit":
                circle = line(stamp, "vision_range", [
                    (pose_xy[0] + self.result.radius *
                     math.cos(2 * math.pi * i / 72),
                     pose_xy[1] + self.result.radius *
                     math.sin(2 * math.pi * i / 72))
                    for i in range(73)], (1.0, 0.1, 0.1, 0.85), 0.25)
                array.markers.append(circle)
            array.markers.append(spheres(stamp, "scan_centers",
                [s.center for s in self.result.snapshots[:scan_index + 1]],
                (0.05, 0.3, 0.8, 1.0), 1.2))
        self.add_bundles(array.markers, stamp, phase)
        robot = marker(stamp, "robot", 0, Marker.SPHERE,
                       (0.05, 0.4, 1.0, 1.0), 1.8)
        robot.pose.position = point(pose_xy, 1.0)
        array.markers.append(robot)
        self.markers_pub.publish(array)

    def tick(self):
        if not self.timeline:
            return
        stamp = self.get_clock().now().to_msg()
        phase, index, pose_xy = self.timeline[self.cursor]
        scan_index = (index if phase == "entry"
                      else len(self.result.snapshots) - 1)
        if not self.map_published:
            self.publish_map(stamp)
            self.map_published = True
        self.publish_markers(stamp, phase, scan_index, pose_xy)

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position = point(pose_xy, 0.6)
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        entry = [s.center for s in self.result.snapshots[:scan_index + 1]]
        self.entry_pub.publish(ros_path(stamp, entry))
        escape = self.result.escape
        if escape and phase != "entry":
            baseline = list(reversed(escape.entry))
            if paper.dist(baseline[-1], escape.return_point) > 1e-6:
                baseline.append(escape.return_point)
            self.baseline_pub.publish(ros_path(stamp, baseline))
            exit_points = escape.exit_path[:index + 1] if phase == "exit" else []
            self.exit_pub.publish(ros_path(stamp, exit_points))
        else:
            self.baseline_pub.publish(ros_path(stamp, []))
            self.exit_pub.publish(ros_path(stamp, []))

        status = String()
        metrics = (f" retrace={escape.entry_length:.2f} "
                   f"dap={escape.exit_length:.2f}") if escape else ""
        status.data = (f"phase={phase} frame={self.cursor + 1}/{len(self.timeline)}"
                       f" goal_invalid={self.goal_invalid}{metrics}")
        self.status_pub.publish(status)
        self.cursor = (self.cursor + 1) % len(self.timeline)


def main():
    rclpy.init()
    node = PaperRvizNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
