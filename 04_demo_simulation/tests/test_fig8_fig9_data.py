"""Checks for the data which the RViz adapter publishes unchanged."""

import ast
import importlib.util
from pathlib import Path
import sys
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "fig8_fig9.py"
sys.path.insert(0, str(SOURCE.parent))
spec = importlib.util.spec_from_file_location("fig8_fig9", SOURCE)
paper = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = paper
spec.loader.exec_module(paper)
from online_mission import OnlineMission, raycast_ranges


class PaperDemoDataTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.result = paper.run_mission(
            paper.START, paper.GOAL, paper.RADIUS, paper.obstacles())

    def test_open_points_are_on_vision_circle(self):
        for snapshot in self.result.snapshots:
            for xy in snapshot.scan.open_points:
                self.assertAlmostEqual(
                    paper.dist(snapshot.center, xy), paper.RADIUS, places=5)

    def test_exit_and_retrace_share_endpoints(self):
        escape = self.result.escape
        self.assertIsNotNone(escape)
        retrace = list(reversed(escape.entry))
        if paper.dist(retrace[-1], escape.return_point) > 1e-6:
            retrace.append(escape.return_point)
        self.assertEqual(retrace[0], escape.exit_path[0])
        self.assertEqual(retrace[-1], escape.exit_path[-1])
        self.assertAlmostEqual(paper.path_length(retrace), escape.entry_length)
        self.assertAlmostEqual(paper.path_length(escape.exit_path),
                               escape.exit_length)
        self.assertLessEqual(escape.exit_length, escape.entry_length + 1e-6)

    def test_original_goal_is_invalid_and_not_claimed_reached(self):
        # This is a known limitation in the paper figure's current scenario.
        self.assertTrue(any(paper.point_in_polygon(paper.GOAL, obstacle)
                            for obstacle in self.result.obstacles))
        self.assertFalse(self.result.reached_goal)

    def test_trimmed_bundles_and_exit_are_collision_free(self):
        escape = self.result.escape
        self.assertGreater(len(escape.crossings), 0)
        self.assertEqual(paper.crossing_pairs(escape.bundles), [])
        for start, end in zip(escape.exit_path, escape.exit_path[1:]):
            self.assertFalse(paper.segment_hits_obstacles(
                start, end, self.result.obstacles))

    def test_rclpy_log_calls_receive_one_message(self):
        scripts = SOURCE.parent / "ros2" / "mars_paper_rviz" / "scripts"
        for node_source in scripts.glob("*.py"):
            tree = ast.parse(node_source.read_text(encoding="utf-8"))
            for call in ast.walk(tree):
                if (isinstance(call, ast.Call) and
                        isinstance(call.func, ast.Attribute) and
                        call.func.attr in {"info", "warn", "error", "debug"} and
                        isinstance(call.func.value, ast.Call) and
                        isinstance(call.func.value.func, ast.Attribute) and
                        call.func.value.func.attr == "get_logger"):
                    self.assertEqual(len(call.args), 1, node_source.name)

    def test_live_mission_moves_then_escapes_in_free_space(self):
        obstacles = paper.obstacles()
        mission = OnlineMission(paper.START, (86.0, 94.0),
                                paper.RADIUS, obstacles)
        self.assertFalse(any(paper.point_in_polygon(mission.goal, obstacle)
                             for obstacle in obstacles))
        plans = []
        for _ in range(12):
            plan = mission.plan_next()
            plans.append(plan)
            self.assertEqual(plan.path[0], plan.scan.center)
            for start, end in zip(plan.path, plan.path[1:]):
                self.assertFalse(paper.segment_hits_obstacles(
                    start, end, obstacles))
            if plan.event == "dap_escape":
                break
        self.assertGreater(len(plans), 1)
        self.assertEqual(plans[-1].event, "dap_escape")
        self.assertGreater(len(plans[-1].escape.crossings), 0)
        self.assertLessEqual(plans[-1].escape.exit_length,
                             plans[-1].escape.entry_length + 1e-6)

    def test_live_mission_rejects_goal_inside_wall(self):
        with self.assertRaises(ValueError):
            OnlineMission(paper.START, paper.GOAL,
                          paper.RADIUS, paper.obstacles())

    def test_lidar_sweep_changes_with_robot_pose(self):
        obstacles = paper.obstacles()
        at_start = raycast_ranges(paper.START, 0.0, paper.RADIUS,
                                  obstacles)
        farther_in = raycast_ranges((33.0, 37.0), 0.0, paper.RADIUS,
                                    obstacles)
        self.assertEqual(len(at_start), 180)
        self.assertNotEqual(at_start, farther_in)
        self.assertTrue(any(reading < paper.RADIUS for reading in at_start))


if __name__ == "__main__":
    unittest.main()
