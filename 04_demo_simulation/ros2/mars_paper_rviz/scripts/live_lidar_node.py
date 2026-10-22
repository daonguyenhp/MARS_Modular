#!/usr/bin/env python3
"""2-D kinematic robot: live raycast LiDAR, incremental planner, RViz output."""

import math
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


sys.path.insert(0, str(Path(get_package_share_directory("mars_paper_rviz")) /
                       "core"))
import fig8_fig9 as paper  # noqa: E402
from online_mission import OnlineMission, raycast_ranges  # noqa: E402


def pt(xy, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(xy[0]), float(xy[1]), float(z)
    return p


def make_marker(stamp, ns, kind, color, size=1.0):
    m = Marker()
    m.header.frame_id = "map"
    m.header.stamp = stamp
    m.ns = ns
    m.id = 0
    m.type = kind
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = float(size)
    m.color.r, m.color.g, m.color.b, m.color.a = color
    return m


def path_msg(stamp, xy_list):
    path = RosPath()
    path.header.frame_id = "map"
    path.header.stamp = stamp
    for xy in xy_list:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position = pt(xy, 0.5)
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)
    return path


class LiveLidarNode(Node):
    PERIOD = 0.1
    RAYS = 180

    def __init__(self):
        super().__init__("live_lidar_robot")
        self.declare_parameter("speed", 8.0)
        self.speed = max(0.1, float(self.get_parameter("speed").value))
        # The figure's original (86,86) is occupied. (86,94) is free and
        # points toward the same alley, so the robot can attempt exploration.
        self.goal = (86.0, 94.0)
        self.mission = OnlineMission(
            paper.START, self.goal, paper.RADIUS, paper.obstacles())
        self.xy = paper.START
        self.yaw = 0.0
        self.route = []
        self.last_plan = None
        self.entry_trace = [self.xy]
        self.exit_trace = []
        self.first_escape = None
        self.phase = "exploring"
        self.map_sent = False

        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched.reliability = ReliabilityPolicy.RELIABLE
        self.map_pub = self.create_publisher(OccupancyGrid, "/paper/map", latched)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/paper/robot_pose", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/paper/markers", 10)
        self.entry_pub = self.create_publisher(
            RosPath, "/paper/travelled_entry", 10)
        self.baseline_pub = self.create_publisher(
            RosPath, "/paper/retrace_baseline", 10)
        self.exit_pub = self.create_publisher(RosPath, "/paper/exit_path", 10)
        self.status_pub = self.create_publisher(String, "/paper/status", latched)
        self.tf = TransformBroadcaster(self)
        self.timer = self.create_timer(self.PERIOD, self.tick)
        self.get_logger().info(
            "Live 2-D LiDAR robot started. Goal is free at (86,94); "
            "stop after the first DAP escape.")

    def send_map(self, stamp):
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.header.stamp = stamp
        grid.info.resolution = 1.0
        grid.info.width = grid.info.height = 100
        grid.info.origin.orientation.w = 1.0
        grid.data = [
            100 if any(paper.point_in_polygon((x + .5, y + .5), obs)
                       for obs in self.mission.obstacles) else 0
            for y in range(100) for x in range(100)
        ]
        self.map_pub.publish(grid)

    def decide(self):
        if self.phase == "finished":
            return
        if self.mission.done:
            self.phase = "finished"
            return
        try:
            plan = self.mission.plan_next()
        except RuntimeError as error:
            self.phase = "finished"
            self.get_logger().warn(f"Planner stopped: {error}")
            return
        self.last_plan = plan
        self.route = list(plan.path[1:])
        self.get_logger().info(
            f"scan {len(self.mission.snapshots)}: {plan.event}, "
            f"target={plan.path[-1]}")
        if plan.escape is not None:
            self.first_escape = plan.escape
            self.exit_trace = [self.xy]
            self.phase = "escaping"
        elif plan.event in {"no_active_open_point", "reached_goal",
                            "invalid_local_edge"}:
            self.phase = "finished"

    def move(self):
        remaining = self.speed * self.PERIOD
        while remaining > 1e-9 and self.route:
            target = self.route[0]
            distance = paper.dist(self.xy, target)
            if distance <= 1e-6:
                self.route.pop(0)
                continue
            self.yaw = math.atan2(target[1] - self.xy[1],
                                  target[0] - self.xy[0])
            distance_moved = min(remaining, distance)
            self.xy = (self.xy[0] + distance_moved * math.cos(self.yaw),
                       self.xy[1] + distance_moved * math.sin(self.yaw))
            remaining -= distance_moved
            if self.phase == "escaping":
                self.exit_trace.append(self.xy)
            else:
                self.entry_trace.append(self.xy)
            if distance_moved >= distance - 1e-6:
                self.xy = target
                self.route.pop(0)
        if not self.route and self.phase == "escaping":
            self.phase = "finished"
            self.mission.done = True
            self.mission.stop_reason = "first_escape_complete"
            self.get_logger().info("Finished first blind-alley DAP escape")

    def send_scan(self, stamp):
        scan = LaserScan()
        scan.header.frame_id = "base_link"
        scan.header.stamp = stamp
        scan.angle_min = -math.pi
        scan.angle_increment = 2 * math.pi / self.RAYS
        scan.angle_max = scan.angle_min + (self.RAYS - 1) * scan.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.PERIOD
        scan.range_min = 0.05
        scan.range_max = self.mission.radius
        scan.ranges = raycast_ranges(
            self.xy, self.yaw, self.mission.radius,
            self.mission.obstacles, self.RAYS)
        self.scan_pub.publish(scan)

    def send_pose_and_tf(self, stamp):
        qz, qw = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = stamp
        pose.pose.position = pt(self.xy, 0.5)
        pose.pose.orientation.z, pose.pose.orientation.w = qz, qw
        self.pose_pub.publish(pose)
        tf = TransformStamped()
        tf.header = pose.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.xy[0]
        tf.transform.translation.y = self.xy[1]
        tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
        self.tf.sendTransform(tf)

    def send_markers(self, stamp):
        markers = MarkerArray()
        clear = make_marker(stamp, "clear", Marker.DELETEALL,
                            (0.0, 0.0, 0.0, 0.0))
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        goal = make_marker(stamp, "valid_goal", Marker.SPHERE,
                           (1.0, 0.1, 0.1, 1.0), 2.0)
        goal.pose.position = pt(self.goal, 0.8)
        markers.markers.append(goal)
        robot = make_marker(stamp, "robot", Marker.ARROW,
                            (0.05, 0.35, 1.0, 1.0), 1.0)
        robot.pose.position = pt(self.xy, 0.8)
        robot.pose.orientation.z = math.sin(self.yaw / 2)
        robot.pose.orientation.w = math.cos(self.yaw / 2)
        robot.scale.x, robot.scale.y, robot.scale.z = 2.6, 1.2, 1.2
        markers.markers.append(robot)
        circle = make_marker(stamp, "lidar_range", Marker.LINE_STRIP,
                             (1.0, 0.2, 0.2, 0.7), 0.18)
        circle.points = [pt((self.xy[0] + self.mission.radius *
                             math.cos(2 * math.pi * i / 72),
                             self.xy[1] + self.mission.radius *
                             math.sin(2 * math.pi * i / 72)), 0.3)
                         for i in range(73)]
        markers.markers.append(circle)
        if self.last_plan is not None:
            points = make_marker(stamp, "open_points", Marker.SPHERE_LIST,
                                 (0.1, 0.85, 0.2, 1.0), 0.9)
            points.points = [pt(xy, 0.7) for xy in
                             self.last_plan.scan.open_points]
            markers.markers.append(points)
        if self.first_escape:
            raw = make_marker(stamp, "raw_bundles", Marker.LINE_LIST,
                              (0.5, 0.5, 0.5, 0.5), 0.12)
            trimmed = make_marker(stamp, "trimmed_bundles", Marker.LINE_LIST,
                                  (0.05, 0.7, 0.95, 0.85), 0.22)
            for bundle_set, target in ((self.first_escape.bundles_raw, raw),
                                       (self.first_escape.bundles, trimmed)):
                for bundle in bundle_set:
                    for a, b in bundle.rays:
                        target.points.extend((pt(a, 0.4), pt(b, 0.4)))
            markers.markers.extend((raw, trimmed))
        self.markers_pub.publish(markers)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        if not self.map_sent:
            self.send_map(stamp)
            self.map_sent = True
        if not self.route and self.phase != "finished":
            self.decide()
        if self.route:
            self.move()
        self.send_pose_and_tf(stamp)
        self.send_scan(stamp)
        self.send_markers(stamp)
        self.entry_pub.publish(path_msg(stamp, self.entry_trace))
        baseline = []
        if self.first_escape:
            baseline = list(reversed(self.first_escape.entry))
            if paper.dist(baseline[-1], self.first_escape.return_point) > 1e-6:
                baseline.append(self.first_escape.return_point)
        self.baseline_pub.publish(path_msg(stamp, baseline))
        self.exit_pub.publish(path_msg(stamp, self.exit_trace))
        status = String()
        status.data = (f"phase={self.phase} pose=({self.xy[0]:.2f},"
                       f"{self.xy[1]:.2f}) scans={len(self.mission.snapshots)} "
                       f"stop_reason={self.mission.stop_reason} goal_reached="
                       f"{self.mission.stop_reason == 'reached_goal'}")
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = LiveLidarNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
