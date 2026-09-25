# Module 3: Navigation Decision

- **WBS:** 3.0, MD3.1, MD3.2, MD3.3
- **Owner:** Member 3
- **Purpose:** Select exploration targets, detect a Blind Alley Region, and
  compute the DAP/funnel escape path.
- **Paper:** *The Sequences of Bundles of Line Segments for Autonomous Robots
  with Limited Vision Range to Escape from Blind Alley Regions*
  (https://doi.org/10.1016/j.robot.2025.105185), Algorithm 2 and Sections 4–5.
- **Robot:** HiWonder JetTank, map-frame metres, path-follower waypoints.
  Core has no ROS, no `cmd_vel`, no LiDAR driver.

## Status

Implemented as a ROS-independent C++17 library. Module 4 still owns launch,
RViz, and the path follower.

## Layout

- `explore/`: MD3.1 pick the highest-ranked reachable open point
- `bar_escape/`: MD3.2 BAR detection and retreat target
- `dap_funnel/`: MD3.3 taut path on Module 2 gates
- `pipeline/`: Algorithm 2 state machine
- `include/`: public headers
- `tests/`, `examples/`

## Build and test

From `MARS_Modular/`:

```sh
cmake -S 03_navigation_decision -B .build/nav -DCMAKE_BUILD_TYPE=Debug
cmake --build .build/nav -j2
ctest --test-dir .build/nav --output-on-failure
```

## Call order (Algorithm 2)

```text
Modules 1–2 produce ranked open points, a skeleton on G_t^e, and gates
        │
        ▼
at goal?                          → GOAL
goal in sight (caller, Eq. 3)?    → direct path to g
highest-rank reachable open point → EXPLORE (last hop capped at max_step)
else BAR                          → retreat to owner concurrent point
                                     or to the entry C_0
ESCAPE                            → funnel on gates, require L_return ≤ L_entry
return reached                    → EXPLORE again
```

Module 3 does **not** compute closed/open sights, ranking, the visibility
graph, or bundle sequences. Those stay in Modules 1 and 2.

## Robot numbers (Mode 1 defaults)

| Parameter | Default | Why |
|---|---|---|
| `vision_radius` | 0.85 m | Paper limited range `r`; Mode 1 clip |
| `max_step` | 0.70 m | Must be `< 2 r` so neighbor sights overlap |
| `goal_tolerance` | 0.15 m | ≥ path-follower waypoint tolerance 0.10 m |
| `arrival_tolerance` | 0.15 m | Same stop band on return |

Output is a polyline in the map frame. Module 4 sends it to the JetTank path
follower.

## API

```cpp
#include "navigator.hpp"
using namespace mars::navigation_decision;
Navigator navigator;  // JetTank defaults
NavigationDecision d = navigator.decide(observation);
```

Synthetic gates are enough to test DAP: see `examples/synthetic_decide.cpp`.
