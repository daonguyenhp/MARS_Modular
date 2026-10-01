# Examples

ROS-free smoke demos for MD4.4. Each file stays under 50 lines.

| File | What it shows |
|---|---|
| `synthetic_perceive.cpp` | One wall-U map → closed/open sights and open points |

Build with Module 1 tests enabled:

```sh
cmake -S 01_perception_geometry -B .build/core -DCMAKE_BUILD_TYPE=Debug
cmake --build .build/core --target mars_perception_geometry_example -j2
.build/core/mars_perception_geometry_example
```

On Windows the binary is under `.build/core/Debug/` or `Release/`.
