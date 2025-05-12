cmake_minimum_required(VERSION 3.5)
project(example_waypoint_flier_simple)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
  mrs_msgs
  geometry_msgs
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  WaypointFlierSimple
  )

include_directories(
  ${colcon_INCLUDE_DIRS}
  )

# WaypointFlierSimple

## Declare a C++ library
add_library(WaypointFlierSimple SHARED
  src/waypoint_flier_simple.cpp
  )

# ament manages cmake related variables and dependency search (similar to catkin in ROS1)
ament_target_dependencies(WaypointFlierSimple
  ${DEPENDENCIES}
  )

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(WaypointFlierSimple PLUGIN "example_waypoint_flier_simple::WaypointFlierSimple" EXECUTABLE WaypointFlierSimple)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
