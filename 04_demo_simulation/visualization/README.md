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

The headless run still writes `mode1_<map>.svg` and the same layer names in the text log.
