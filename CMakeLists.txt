cmake_minimum_required(VERSION 3.8)
project(dubin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(mission_planner src/MissionPlanner.cpp
src/collision_check.cpp
src/config_server.cpp
src/DPRMstar.cpp
src/dubinCurve.cpp
src/ExactCell.cpp
src/GeometricPRMstar.cpp
src/graph.cpp
src/map.cpp
src/Planner.cpp
#src/ApproximateCellDecomposition.cpp
)

ament_target_dependencies(mission_planner rclcpp nav_msgs tf2 tf2_ros geometry_msgs nav2_msgs rclcpp_action obstacles_msgs std_msgs) 

install(TARGETS
  mission_planner
  DESTINATION lib/${PROJECT_NAME}/)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

ament_package()
