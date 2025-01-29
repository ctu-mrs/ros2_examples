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
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_examples REQUIRED)
find_package(example_plugin_manager REQUIRED)

# export the plugins listed in plugins.xml to be discovered by the manager
pluginlib_export_plugin_description_file(example_plugin_manager plugins.xml)

set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# ${PROJECT_NAME}
add_library(${PROJECT_NAME} SHARED
  src/example_plugin.cpp
  )

# ament manages cmake related variables and dependency search (similar to catkin in ROS1)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  rclcpp
  rclcpp_components
  Eigen3
  pluginlib
  ros2_examples
  example_plugin_manager
)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(
  TARGETS ${PROJECT_NAME} # list all libraries to be installed and exported out from this pkg
  EXPORT export_${PROJECT_NAME} # a single export name is enough to export all libraries from this pkg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
# export the installed library and related info to the cmake system
# this is ESSENTIAL for the plugin to be discovered dynamically
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()