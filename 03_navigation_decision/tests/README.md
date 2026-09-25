# Navigation Decision Tests

`test_navigation_decision.cpp` covers MD3.1–3.3 and the Algorithm 2 machine
with synthetic ranked points and synthetic gates. No ROS, no robot, no map file.

```sh
cmake -S 03_navigation_decision -B .build/nav
cmake --build .build/nav
ctest --test-dir .build/nav --output-on-failure
```
