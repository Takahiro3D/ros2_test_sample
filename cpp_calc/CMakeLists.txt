cmake_minimum_required(VERSION 3.8)
project(cpp_calc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(twice_lib
  SHARED
  src/twice.cpp
  src/twice_node.cpp
)

ament_auto_add_executable(twice
  src/main.cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_auto_find_test_dependencies()
  ament_auto_add_gtest(twice_test
    test/unit/twice_test.cpp
  )

  add_launch_test(test/launch/twice_node_test.py
    RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py"
  )
endif()

ament_auto_package()