
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

| Test size | Test framework | package |
| --- | --- | --- |
| Small | lint / gtest / pytest | Each node packages |
| Medium | launch_testing | Each node packages |
| Large | launch_pytest | integration |

# Reference

https://speakerdeck.com/fixstars/autonomous-driving-realization-by-ros2-test-framework?slide=159
