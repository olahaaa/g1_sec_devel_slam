cmake_minimum_required(VERSION 3.8)
project(fix_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(imu_fix_node src/imu_fix_node.cpp)
add_executable(cloud_fix_node src/cloud_fix_node.cpp)

ament_target_dependencies(imu_fix_node rclcpp sensor_msgs)
ament_target_dependencies(cloud_fix_node rclcpp sensor_msgs)

install(
  TARGETS 
  imu_fix_node
  cloud_fix_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/measure_imu_delay.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
