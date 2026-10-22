"""Checks for the data which the RViz adapter publishes unchanged."""

import importlib.util
from pathlib import Path
import sys
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "fig8_fig9.py"
spec = importlib.util.spec_from_file_location("fig8_fig9", SOURCE)
paper = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = paper
spec.loader.exec_module(paper)


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


if __name__ == "__main__":
    unittest.main()
