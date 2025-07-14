# cmake_minimum_required(VERSION 3.8)
# project(traversability_mapping)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)
# find_package(pcl_ros REQUIRED)
# find_package(tf2_ros REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()




cmake_minimum_required(VERSION 3.8)
project(traversability_mapping VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(elevation_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(PCL REQUIRED QUIET)
find_package(pcl_ros REQUIRED)
find_package(OpenCV REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)


include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


# Range Image Filter
add_executable(traversability_filter
  src/traversability_filter.cpp
)
ament_target_dependencies(traversability_filter
  rclcpp
  sensor_msgs
  elevation_msgs
  pcl_conversions
  pcl_ros
  OpenCV
)

# Mapping
add_executable(traversability_map
  src/traversability_map.cpp
)
ament_target_dependencies(traversability_map
  rclcpp
  sensor_msgs
  nav_msgs
  elevation_msgs
  pcl_conversions
  pcl_ros
  OpenCV
  cv_bridge
  image_transport
)

# PRM Planner
add_executable(traversability_prm
  src/traversability_prm.cpp
  src/library/kdtree.c
)
ament_target_dependencies(traversability_prm
  rclcpp
  sensor_msgs
  geometry_msgs
  elevation_msgs
  pcl_conversions
  pcl_ros
  OpenCV
)

# Path Planner
add_executable(traversability_path
  src/traversability_path.cpp
)
ament_target_dependencies(traversability_path
  rclcpp
  sensor_msgs
  geometry_msgs
  elevation_msgs
  pcl_conversions
  pcl_ros
  OpenCV
)


# move_base Planner Plugin
add_library(tm_planner_lib SHARED
  src/library/tm_planner.cpp
)
ament_target_dependencies(tm_planner_lib
  pluginlib
  nav2_core
  rclcpp
)

install(FILES plugin/tm_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME}/plugins
)
pluginlib_export_plugin_description_file(nav2_core plugins/tm_planner_plugins.xml)

install(TARGETS
  traversability_filter
  traversability_map
  traversability_prm
  traversability_path
  tm_planner_lib
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()