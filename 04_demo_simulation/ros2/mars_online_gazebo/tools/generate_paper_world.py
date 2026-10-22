#!/usr/bin/env python3
"""Generate the Gazebo collision world from the paper sketch's walls only.

This imports obstacle coordinates from fig8_fig9.py, never its path/planner.
One sketch unit is 0.1 m. The generated SDF is committed for ROS runtime.
"""

import ast
from pathlib import Path
from xml.etree import ElementTree as ET


HERE = Path(__file__).resolve().parent
SCALE = 0.1
# The sketch's (86,86) lies inside a wall; (86,68) is in free space.
GOAL = (8.6, 6.8)


def sketch_geometry():
    """Read only literal wall coordinates, without importing sketch/plot code."""
    source = HERE.parents[2] / "fig8_fig9.py"
    tree = ast.parse(source.read_text(encoding="utf-8"))
    start = None
    rectangles = None
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and getattr(node.target, "id", "") == "START":
            start = ast.literal_eval(node.value)
        if isinstance(node, ast.FunctionDef) and node.name == "obstacles":
            returned = next(item.value for item in node.body if isinstance(item, ast.Return))
            rectangles = []
            for call in returned.elts:
                if not isinstance(call, ast.Call) or getattr(call.func, "id", "") != "rect":
                    raise ValueError("expected only literal rect(...) walls")
                x0, y0, x1, y1 = (ast.literal_eval(arg) for arg in call.args)
                rectangles.append((x0, y0, x1, y1))
    if start is None or rectangles is None:
        raise ValueError("missing paper START / obstacles")
    return start, rectangles


def wall(name, rect):
    x0, y0, x1, y1 = (value * SCALE for value in rect)
    sx, sy = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    return f"""  <model name=\"{name}\"><static>true</static>
    <pose>{cx:.3f} {cy:.3f} 0.30 0 0 0</pose>
    <link name=\"wall\">
      <collision name=\"solid\"><geometry><box><size>{sx:.3f} {sy:.3f} 0.60</size></box></geometry></collision>
      <visual name=\"gray_wall\"><geometry><box><size>{sx:.3f} {sy:.3f} 0.60</size></box></geometry>
        <material><ambient>0.48 0.53 0.58 1</ambient><diffuse>0.48 0.53 0.58 1</diffuse></material>
      </visual>
    </link>
  </model>"""


def generate():
    sketch_start, obstacles = sketch_geometry()
    start = (sketch_start[0] * SCALE, sketch_start[1] * SCALE)
    assert len(obstacles) == 16
    for point in (start, GOAL):
        sketch_point = (point[0] / SCALE, point[1] / SCALE)
        assert not any(x0 <= sketch_point[0] <= x1 and y0 <= sketch_point[1] <= y1
                       for x0, y0, x1, y1 in obstacles), f"pose inside wall: {point}"
    walls = "\n".join(wall(f"paper_wall_{i:02d}", polygon)
                      for i, polygon in enumerate(obstacles))
    template = (HERE.parent / "worlds" / "paper_alley.template.sdf").read_text(
        encoding="utf-8")
    result = template.replace("<!-- PAPER_WALLS -->", walls)
    result = result.replace("__START_X__", str(start[0]))
    result = result.replace("__START_Y__", str(start[1]))
    ET.fromstring(result)
    return result


if __name__ == "__main__":
    output = HERE.parent / "worlds" / "paper_alley.sdf"
    output.write_text(generate(), encoding="utf-8")
    print(f"wrote {output}")
