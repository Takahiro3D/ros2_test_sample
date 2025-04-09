
# ROS2 test sample 

ROS2 test sample project

# Build

``` bash
colcon build --symlink-install
```

# Test

``` bash
colcon test
```

The test executes following cases.

| Test size | Test framework | package | single test sample |
| --- | --- | --- | --- |
| Small | lint / gtest / pytest | Each node packages | colcon test --packages-select cpp_calc --ctest-args -R twice_test |
| Medium | launch_testing | Each node packages | colcon test --packages-select cpp_calc --ctest-args -R twice_node_test |
| Large | launch_pytest | integration | colcon test --packages-select integration |

# Debug

## Debug build

``` bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Single process debugging

Launch with gdb debbugger

For example, use F5 "(gdb) Launch"

## Launch debugging

Launch to use ros launch

For example, use F5 "ROS: Launch pubsub"

# Reference

[ROS2自律走行実現に向けて2 / Autonomous driving realization by ROS2 test framework](https://speakerdeck.com/fixstars/autonomous-driving-realization-by-ros2-test-framework)
