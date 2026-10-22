# Module 4: Demo and Simulation

## Figure 8/9 Python simulation in ROS 2 RViz

`ros2/mars_paper_rviz` plays back the **existing** `fig8_fig9.py` result. It
publishes the 100-by-100 scenario obstacle map, robot pose, accumulated
open/closed sights, open-point rays, raw/trimmed bundles, crossings, the actual
path into the alley, the retrace baseline, and the DAP exit path. It does not
run Gazebo or a physical robot. The map is ground truth; the sight overlay is
what the simulated robot has scanned so far.

On Ubuntu 26.04 / ROS 2 Lyrical, with this repository at
`~/hcmut/ros2_ws/mars_ws/src/MARS_Modular`:

```bash
source /opt/ros/lyrical/setup.bash
cd ~/hcmut/ros2_ws/mars_ws
colcon build --base-paths src/MARS_Modular/04_demo_simulation/ros2 \
  --packages-select mars_paper_rviz --symlink-install
source install/setup.bash
ros2 launch mars_paper_rviz paper_fig8_fig9.launch.py
```

For a slower playback, append `playback_hz:=0.7`. If RViz is already open, use
`run_rviz:=false` and load
`04_demo_simulation/ros2/mars_paper_rviz/rviz/paper_fig8_fig9.rviz` there.
The RViz fixed frame is `map`. Useful checks in a second sourced terminal:

```bash
ros2 topic list | grep /paper/
ros2 topic echo /paper/status
```

The original scenario has `GOAL=(86,86)` **inside an obstacle**. The node
warns about this rather than claiming goal-reaching. The Python script stops
at the first blind-alley escape, so this playback demonstrates Figure 8/9,
not the complete robot-to-goal run. The two reported lengths compare the same
endpoints: dead end to return point by retracing (`/paper/retrace_baseline`)
versus DAP (`/paper/exit_path`). `/paper/travelled_entry` is the actual forward
route into the alley; it is not the path used for that length comparison.

This ROS 2 adapter is independent of the older Mode 1 C++ playback below.
It has no Matplotlib runtime dependency; Matplotlib is needed only to render
the PNG from `fig8_fig9.py`.

## Existing Mode 1 demo (ROS 1 default)

Opens RViz and plays the `geogebra` map. From `MARS_Modular/`:

```sh
./04_demo_simulation/run.sh
```

`--map hard_alley` selects the other map. `--svg` writes `mode1_<map>.svg` without a window.

Figure 8 and Figure 9 of the paper, as one PNG, without ROS:

```text
pip install matplotlib
python 04_demo_simulation/fig8_fig9.py
```

The image is `04_demo_simulation/output/fig8_fig9.png`.
