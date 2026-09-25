# Launch

`polygon_explore_sim` loads a Mode 1 map, then repeats:

1. Module 1 perceives the pose.
2. Module 2 updates the graph and bundles.
3. Module 3 plans a path.
4. The fake robot integrates follower `cmd_vel` until that path is done.

```sh
polygon_explore_sim --list
polygon_explore_sim --map hard_alley
polygon_explore_sim --map geogebra
```

The follower stops at 0.10 m on intermediate waypoints and 0.08 m on the last point. The planner accepts a pose inside 0.15 m, so the two stop bands do not freeze the robot. A headless run also writes `mode1_<map>.svg` and prints the marker log.

The RViz window is the ROS 2 launch `mars_mode1_sim polygon_explore_sim.launch.py`.
