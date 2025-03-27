cmake_minimum_required(VERSION 3.5)
project(example_plugins)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  rclcpp_components
  Eigen3
  pluginlib
  ros2_examples
  example_plugin_manager
)

set(LIBRARIES
  ExamplePlugin_Plugin
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${example_plugin_manager_INCLUDE_DIRS}
  )

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# Example plugin

add_library(ExamplePlugin_Plugin SHARED
  src/example_plugin.cpp
  )

# ament manages cmake related variables and dependency search (similar to catkin in ROS1)
ament_target_dependencies(ExamplePlugin_Plugin
  ${DEPENDENCIES}
)

## --------------------------------------------------------------
## |                       Export plugins                       |
## --------------------------------------------------------------

# <package of the base class>, <relative path to the xml>
pluginlib_export_plugin_description_file(example_plugin_manager plugins.xml)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# export the installed library and related info to the cmake system
# this is ESSENTIAL for the plugin to be discovered dynamically
ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
