# Online Gazebo + RViz paper alley

This package is an **online, sensor-driven experiment**, not the earlier RViz
playback and not `fig8_fig9.py`'s path calculation. `paper_alley.sdf` contains
the exact 16 rectangle coordinates from `fig8_fig9.py`, scaled by 0.1 m per
sketch unit. Python is used only by `tools/generate_paper_world.py` to extract
those literal wall coordinates and generate the SDF. The planner does not read
the SDF, occupancy grid, original sketch path, or a precomputed entry/exit.

Gazebo spawns the official TurtleBot3 Burger model with collision geometry and
its LiDAR/differential-drive plugins. Gazebo publishes `/scan` and `/odom`; `ros_gz_bridge` passes them
to ROS 2. The ROS node converts each *current* LaserScan into visible wall
segments, calls the repository's Module 1 perception, Module 2 graph/bundle
manager, and Module 3 navigator, then follows only the next selected path with
`/cmd_vel`. After driving that path it scans and decides again. A low-level
safety recovery is enabled: when the forward LiDAR sector is too close, the
controller backs up, turns toward the more open side, waits for a fresh scan,
and replans. BAR/DAP/Funnel remains the MARS decision logic. RViz shows a
LiDAR-built occupancy grid for inspection, not for planning.

## Ubuntu run

Prerequisites: ROS 2 Lyrical, Gazebo Jetty, `ros_gz_sim`, `ros_gz_bridge`, RViz 2,
the ROS 2 `turtlebot3_gazebo` package, and `colcon`. In the workspace containing
this repository:

```bash
cd ~/hcmut/ros2_ws/mars_ws/src/MARS_Modular
git fetch origin
git switch --track origin/codex/gazebo-online-sim
cd ~/hcmut/ros2_ws/mars_ws
source /opt/ros/lyrical/setup.bash
colcon build --base-paths src/MARS_Modular/04_demo_simulation/ros2 \
  --packages-select mars_online_gazebo --symlink-install
source install/setup.bash
ros2 launch mars_online_gazebo online_paper_alley.launch.py
```

If that local branch already exists, use `git switch codex/gazebo-online-sim`
and `git pull --ff-only` instead. Do not use `git switch --track` a second time.
Gazebo must be **playing**. If your VM cannot render the GPU LiDAR with Ogre2,
check `ros2 topic hz /scan`; the node deliberately waits instead of moving.

The launch arguments `start_x/start_y` and `goal_x/goal_y` are expressed in
world coordinates. Gazebo's odometry starts at `(0,0)` at the spawn pose, so
the launch passes the spawn offset to the node and converts the goal into the
odom frame automatically. The default goal is `(8.6, 6.8)` m. The original Figure 8/9 goal `(86,86)`
sketch units lies *inside* an obstacle; it cannot be used as a physical robot
destination. Use RViz's `2D Goal Pose` tool to send a different free-space goal
on `/goal_pose` (frame `odom`) and reset the online planner.

## Check that it really runs online

In separate terminals with the workspace sourced:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic echo /mars_online/status
ros2 topic echo /cmd_vel
```

You should see live LiDAR, changing odometry, changing motion commands, and
`EXPLORE` / `FUNNEL` / `GOAL_VISIBLE` decisions. Robot geometry is in Gazebo;
RViz shows scan, growing observed map, the pose marker, paper vision circle,
Module 2 bundle rays, separate orange entry / blue return tracks, and current
plan. `WAIT_SENSORS`,
`FUNNEL_FAIL`, `FRONTIER_EXHAUSTED`, `SAFETY_STOP`, `RECOVERY_BACKOFF`,
`RECOVERY_TURN`, `REPLAN`, `SAFETY_BLOCKED`, and `MOTION_STALLED` are diagnostic
states, not successful escape claims. `SAFETY_BLOCKED` means five recovery
attempts failed. This is an initial integration for
Ubuntu runtime verification; it has not yet been exercised against Gazebo
from the Windows development machine.

For a persistent per-scan trace, pass a CSV path to the node:

```bash
ros2 run mars_online_gazebo online_navigation_node --ros-args \
  -p use_sim_time:=true -p goal_x:=8.6 -p goal_y:=6.8 \
  -p status_log:=/tmp/mars_test_A.csv
```

The CSV records pose, event, LiDAR clearance in front/left/right, open-point
count, planned waypoint count, and recovery phase. It is flushed as the run
continues, so it can be copied after stopping the experiment.

To regenerate the same wall geometry after changing the sketch:

```bash
python3 src/MARS_Modular/04_demo_simulation/ros2/mars_online_gazebo/tools/generate_paper_world.py
```
