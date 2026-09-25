# Maps

Mode 1 reads these occupancy files. Wall coordinates are not stored in C++.

| id | file | start | goal |
|---|---|---|---|
| `hard_alley` | `hard_alley_map.yaml` + `.pgm` | (0.35, 0.35) | (0.35, 3.40) |
| `geogebra` | `geogebra_map.yaml` + `.pgm` | (0.35, 0.35) | (0.35, 2.90) |

`load_occupancy_map` turns occupied cells into polygons. `polygon_explore_sim` passes those polygons to Module 1, then Module 2, then Module 3. Start and goal are the launch arguments from `polygon_explore_sim.launch`.
