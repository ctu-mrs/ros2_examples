cmake_minimum_required(VERSION 3.5)
project(ros2_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(mrs_lib REQUIRED)

## --------------------------------------------------------------
## |                   service server example                   |
## --------------------------------------------------------------

# specify a new target to be compiled (a dynamic library in this case)
add_library(service_server_example SHARED
  src/service_server_example.cpp
)

# ensures that headers and .so's are available when compiling the targe
ament_target_dependencies(service_server_example
  rclcpp
  rclcpp_components
  std_srvs
  mrs_lib
)

# registers the passed class from the specified library as a ROS2 component
rclcpp_components_register_nodes(service_server_example PLUGIN "ros2_examples::ServiceServerExample" EXECUTABLE service_server_example)

## --------------------------------------------------------------
## |                   service client example                   |
## --------------------------------------------------------------

# specify a new target to be compiled (a dynamic library in this case)
add_library(service_client_example SHARED
  src/service_client_example.cpp
)

# ensures that headers and .so's are available when compiling the targe
ament_target_dependencies(service_client_example
  rclcpp
  rclcpp_components
  std_srvs
  mrs_lib
)

# registers the passed class from the specified library as a ROS2 component
rclcpp_components_register_nodes(service_client_example PLUGIN "ros2_examples::ServiceClientExample" EXECUTABLE service_client_example)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  service_server_example service_client_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
