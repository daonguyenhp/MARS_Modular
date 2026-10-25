"""Verify the generated SDF really reproduces the paper sketch walls."""

from collections import deque
from pathlib import Path
from xml.etree import ElementTree as ET
import importlib.util


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("generate_paper_world", ROOT / "tools" / "generate_paper_world.py")
generator = importlib.util.module_from_spec(spec)
spec.loader.exec_module(generator)


def test_world():
    start, rectangles = generator.sketch_geometry()
    generated = generator.generate()
    assert generated == (ROOT / "worlds" / "paper_alley.sdf").read_text(encoding="utf-8")
    xml = ET.fromstring(generated)
    walls = [model for model in xml.findall(".//model")
             if model.get("name", "").startswith("paper_wall_")]
    assert len(walls) == len(rectangles) == 16
    for model, (x0, y0, x1, y1) in zip(walls, rectangles):
        x, y, _z, *_ = map(float, model.find("pose").text.split())
        assert abs(x - (x0 + x1) * 0.05) < 1e-6
        assert abs(y - (y0 + y1) * 0.05) < 1e-6

    # Robot radius + small clearance. This checks that the adjusted goal is
    # physically reachable, not that the online MARS planner will find it.
    clearance = 1.6  # sketch units
    def free(cell):
        x, y = cell[0] * 0.5, cell[1] * 0.5
        return all(not (x0 - clearance <= x <= x1 + clearance and
                        y0 - clearance <= y <= y1 + clearance)
                   for x0, y0, x1, y1 in rectangles)

    source = tuple(round(c * 2) for c in start)
    target = tuple(round(c * 20) for c in generator.GOAL)
    assert free(source) and free(target)
    reached = {source}
    queue = deque([source])
    while queue and target not in reached:
        x, y = queue.popleft()
        for next_cell in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if (0 <= next_cell[0] <= 200 and 0 <= next_cell[1] <= 200 and
                    next_cell not in reached and free(next_cell)):
                reached.add(next_cell)
                queue.append(next_cell)
    assert target in reached


if __name__ == "__main__":
    test_world()
    print("world geometry and free-space reachability passed")
