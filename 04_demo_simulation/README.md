# Module 4: Demo and Simulation

- **WBS:** 4.0, MD4.1, MD4.2, MD4.3, MD4.4
- **Owner:** Shared integration responsibility
- **Purpose:** Run the pure modules on a Mode 1 map without ROS.
- **Acceptance:** one command loads the map, the fake robot follows each planned path, and RViz shows the marker layers.

## Run

From `MARS_Modular/`:

```sh
cmake -S . -B .build/mars -DCMAKE_BUILD_TYPE=Debug
cmake --build .build/mars --target polygon_explore_sim -j2
.build/mars/04_demo_simulation/polygon_explore_sim --map hard_alley
.build/mars/04_demo_simulation/polygon_explore_sim --map geogebra
```

On Windows the executable is under `.build/mars/04_demo_simulation/Debug/` or `Release/`.

RViz on the robot stack is ROS 1 Noetic. The package is `04_demo_simulation/ros1/mars_mode1_sim`. It links modules 1–3 and publishes `/map`, `/dsfm_online/polygon_markers`, `/planned_path`, and `/robot_current_pose`.

```sh
mkdir -p ~/mars_ws/src
ln -sfn /mnt/f/02_IVS_IMACS/01_JetTank/JetTank_runtime_bundle/hiwonder/modular/MARS_Modular/04_demo_simulation/ros1/mars_mode1_sim ~/mars_ws/src/mars_mode1_sim
cd ~/mars_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch mars_mode1_sim polygon_explore_sim.launch map:=hard_alley
```

`map:=geogebra` selects the other map. `run_rviz:=false` keeps the topics without the window.

```sh
ctest --test-dir .build/mars -R "mode1|polygon_explore_sim" --output-on-failure
```
