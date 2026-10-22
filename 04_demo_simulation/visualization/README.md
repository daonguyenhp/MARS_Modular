# Visualization

RViz is the Mode 1 viewer. The ROS 1 node in `ros1/mars_mode1_sim` publishes:

| Topic | What |
|---|---|
| `/map` | occupancy grid |
| `/dsfm_online/polygon_markers` | walls, goal, trail, vision circle, closed/open sights, open points, visibility graph, bundle sequence, funnel |
| `/planned_path` | path of the current sensing pose |
| `/robot_current_pose` | robot arrow |
| `/dsfm_online/status` | event name |

```sh
roslaunch mars_mode1_sim polygon_explore_sim.launch map:=hard_alley
```

The headless run writes `04_demo_simulation/output/mode1_<map>.svg` from the same layers: vision circles, closed and open sights, open points, the visibility graph, bundle rays, the driven trail, and the funnel. The text log lists those layer names.
