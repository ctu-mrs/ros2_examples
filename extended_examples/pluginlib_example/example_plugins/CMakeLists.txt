cmake_minimum_required(VERSION 3.5)
project(example_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_examples REQUIRED)
find_package(example_plugin_manager REQUIRED)
find_package(Eigen3 REQUIRED)

pluginlib_export_plugin_description_file(example_plugin_manager plugins.xml)

set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

include_directories(
  include
  )

# ExamplePlugin

add_library(example_plugin SHARED
  src/example_plugin.cpp
  )

# ament manages cmake related variables and dependency search (similar to catkin in ROS1)
ament_target_dependencies(example_plugin PUBLIC
  rclcpp
  rclcpp_components
  pluginlib
  Eigen3
  ros2_examples
  example_plugin_manager
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(example_plugin "example_plugin::ExamplePlugin")

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS
  example_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()