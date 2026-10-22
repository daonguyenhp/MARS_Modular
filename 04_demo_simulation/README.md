# Module 4: Demo and Simulation

The picture a customer can keep is drawn from modules 1–3. `polygon_explore_sim` perceives, updates the graph and bundles, plans, follows the path, then writes `mode1_<map>.svg`.

From `MARS_Modular/` on a machine with ROS 1 Noetic, this opens RViz on the `geogebra` map:

```sh
./04_demo_simulation/run.sh
```

`--map hard_alley` selects the other map. `--svg` does not open a window. It builds the sim and writes:

- `mode1_<map>.svg` in the current directory
- `04_demo_simulation/output/mode1_<map>.svg`

On Windows, from `MARS_Modular/`:

```powershell
cmake -S . -B .build/mars -DCMAKE_BUILD_TYPE=Debug
cmake --build .build/mars --target polygon_explore_sim -j2
.build\mars\04_demo_simulation\Debug\polygon_explore_sim.exe --map geogebra
.build\mars\04_demo_simulation\Debug\polygon_explore_sim.exe --map hard_alley
```

The SVG contains the walls, vision circles, closed sights, open sights, open points, visibility graph, bundle rays, the driven trail, and the funnel. Send that file. Do not send `output/fig8_fig9.png`: `fig8_fig9.py` draws a paper sketch with its own geometry and does not call modules 1–3.

The paper maze, drawn by modules 1–3 as four separate pictures:

```powershell
.\.build\mars\04_demo_simulation\Debug\polygon_explore_sim.exe --map fig8
```

This writes `fig8a_sights.svg`, `fig8b_graph.svg`, `fig9b_bundles.svg`, and `fig9c_exit.svg` into `04_demo_simulation/output/`. Each file shows one layer. The map, start `(30, 22)`, goal `(86, 86)`, and radius `16` match the sketch. The paths are the C++ modules, so they will not match the sketch point for point.
