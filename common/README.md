# Common Contracts

This directory contains ROS-independent data models and interfaces shared by the pure modules. Keep shared types lightweight and avoid importing ROS, simulation, or runtime-specific packages.

- `types/`: data contracts used across module boundaries.
- `interfaces/`: protocol and callable skeletons for future implementations.

`cpp/include/mars/common/geometry_types.hpp` defines canonical ROS-independent
C++17 geometry primitives. Module 1's C++ perception result types live in its
public `types.hpp` header. Python models/protocols remain interface placeholders;
no language binding is implemented. C++ angular intervals use start/sweep,
merged closed sights carry multiple visible boundaries, and open points include
their representative angle.
