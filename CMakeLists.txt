cmake_minimum_required(VERSION 3.5)
project(ros2_uav_example)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

# include_directories(
#   include
# )

add_library(ros2_uav_example SHARED
  src/example.cpp
)

target_compile_definitions(ros2_uav_example
  PRIVATE "ROS2_UAV_EXAMPLE_BUILDING_DLL")

ament_target_dependencies(ros2_uav_example
  rclcpp
  rclcpp_components
  std_msgs
)

# target_link_libraries(ros2_uav_example
# )

rclcpp_components_register_nodes(ros2_uav_example PLUGIN "ros2_uav_example::Example" EXECUTABLE example)

# set(node_plugins "${node_plugins}${PROJECT_NAME}::Example\n")

install(TARGETS ros2_uav_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
