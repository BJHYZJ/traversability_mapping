# cmake_minimum_required(VERSION 2.8.3)
# project(elevation_msgs)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

# find_package(catkin REQUIRED COMPONENTS
#   message_generation
#   geometry_msgs
#   std_msgs
#   nav_msgs
# )


# ## Generate messages in the 'msg' folder
# add_message_files(
#   DIRECTORY msg
#   FILES
#   #MapMetaData.msg
#   #OccupancyGrid.msg
#   OccupancyElevation.msg
# )

# #add_service_files(
# #  DIRECTORY srv
# #  FILES
# #  empty.srv)



# ## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   geometry_msgs
#   std_msgs
#   nav_msgs
# )


# catkin_package(
# #  INCLUDE_DIRS include
# #  LIBRARIES elevation_msgs
#   CATKIN_DEPENDS 
#   message_runtime 
#   message_generation
#   geometry_msgs 
#   std_msgs
#   nav_msgs
# )

# ###########
# ## Build ##
# ###########

# include_directories(
#   ${catkin_INCLUDE_DIRS}
# )


cmake_minimum_required(VERSION 3.5)
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
find_package(nav_core REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

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
)

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

add_library(tm_planner_lib SHARED
  src/library/tm_planner.cpp
)
ament_target_dependencies(tm_planner_lib
  pluginlib
  nav_core
  rclcpp
)

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
