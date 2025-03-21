cmake_minimum_required(VERSION 3.5)
project(example_plugin_manager)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(DEPENDENCIES
  ament_cmake
  ament_cmake_ros
  rclcpp
  rclcpp_components
  pluginlib
  Eigen3
  ros2_examples
)

set(LIBRARIES
  ExamplePluginManager_PluginManager
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_examples_INCLUDE_DIRS}
  )

# ExamplePluginManager
add_library(ExamplePluginManager_PluginManager SHARED
 src/example_plugin_manager.cpp
 )

# ament manages cmake related variables and dependency search (similar to catkin in ROS1)
ament_target_dependencies(ExamplePluginManager_PluginManager
  ${DEPENDENCIES}
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(ExamplePluginManager_PluginManager "example_plugin_manager::ExamplePluginManager")

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
  DIRECTORY include
  DESTINATION .
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
