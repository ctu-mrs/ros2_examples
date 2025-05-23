cmake_minimum_required(VERSION 3.5)
project(example_waypoint_flier)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 0 == use MRS's thread timer implementation
# 1 == use rclcpp::Timer implementation
set(USE_ROS_TIMER 0)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  # dynamic_reconfigure
  geometry_msgs
  nav_msgs
  mrs_msgs
  mrs_lib
  Eigen3
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  WaypointFlier
  )

## Generate dynamic parameters which are loaded by the dynamic reconfigure server
# generate_dynamic_reconfigure_options(
#   config/dynparam.cfg
#   )

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${colcon_INCLUDE_DIRS}
  ${mrs_lib_INCLUDE_DIRS}
  )

## Declare a C++ library
add_library(WaypointFlier SHARED
  src/example_waypoint_flier.cpp
  )

# ament manages cmake related variables and dependency search (similar to catkin in ROS1)
ament_target_dependencies(WaypointFlier
  ${DEPENDENCIES}
  )

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(WaypointFlier PLUGIN "example_waypoint_flier::ExampleWaypointFlier" EXECUTABLE WaypointFlier)

target_compile_definitions(WaypointFlier PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

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
