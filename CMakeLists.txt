cmake_minimum_required(VERSION 3.5)
project(ros2_examples)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# include_directories(
#   include
# )

## --------------------------------------------------------------
## |                     subscriber example                     |
## --------------------------------------------------------------

add_library(subscriber_example SHARED
  src/subscriber_example.cpp
)

# TODO WHY IS THIS HERE?
target_compile_definitions(subscriber_example
  PRIVATE "ros2_examples_BUILDING_DLL")

ament_target_dependencies(subscriber_example
  rclcpp
  rclcpp_components
  std_msgs
)

# target_link_libraries(subscriber_example
# )

# TODO HOW DOES THIS WORK?
rclcpp_components_register_nodes(subscriber_example PLUGIN "ros2_examples::SubscriberExample" EXECUTABLE subscriber_example)

# set(node_plugins "${node_plugins}${PROJECT_NAME}::Example\n")

## --------------------------------------------------------------
## |                      publisher example                     |
## --------------------------------------------------------------

add_library(publisher_example SHARED
  src/publisher_example.cpp
)

# TODO WHY IS THIS HERE?
target_compile_definitions(publisher_example
  PRIVATE "ros2_examples_BUILDING_DLL")

ament_target_dependencies(publisher_example
  rclcpp
  rclcpp_components
  std_msgs
)

# target_link_libraries(publisher_example
# )

# TODO HOW DOES THIS WORK?
rclcpp_components_register_nodes(publisher_example PLUGIN "ros2_examples::PublisherExample" EXECUTABLE publisher_example)

# set(node_plugins "${node_plugins}${PROJECT_NAME}::Example\n")

## --------------------------------------------------------------
## |                   service server example                   |
## --------------------------------------------------------------

add_library(service_server_example SHARED
  src/service_server_example.cpp
)

# TODO WHY IS THIS HERE?
target_compile_definitions(service_server_example
  PRIVATE "ros2_examples_BUILDING_DLL")

ament_target_dependencies(service_server_example
  rclcpp
  rclcpp_components
  std_srvs
)

# target_link_libraries(service_server_example
# )

# TODO HOW DOES THIS WORK?
rclcpp_components_register_nodes(service_server_example PLUGIN "ros2_examples::ServiceServerExample" EXECUTABLE service_server_example)

# set(node_plugins "${node_plugins}${PROJECT_NAME}::ServiceServerExample\n")

## --------------------------------------------------------------
## |                   service client example                   |
## --------------------------------------------------------------

add_library(service_client_example SHARED
  src/service_client_example.cpp
)

# TODO WHY IS THIS HERE?
target_compile_definitions(service_client_example
  PRIVATE "ros2_examples_BUILDING_DLL")

ament_target_dependencies(service_client_example
  rclcpp
  rclcpp_components
  std_srvs
)

# target_link_libraries(service_client_example
# )

# TODO HOW DOES THIS WORK?
rclcpp_components_register_nodes(service_client_example PLUGIN "ros2_examples::ServiceClientExample" EXECUTABLE service_client_example)

# set(node_plugins "${node_plugins}${PROJECT_NAME}::ServiceClientExample\n")

## --------------------------------------------------------------
## |                        timer example                       |
## --------------------------------------------------------------

add_library(timer_example SHARED
  src/timer_example.cpp
)

# TODO WHY IS THIS HERE?
target_compile_definitions(timer_example
  PRIVATE "ros2_examples_BUILDING_DLL")

ament_target_dependencies(timer_example
  rclcpp
  rclcpp_components
  std_srvs
)

# target_link_libraries(timer_example
# )

# TODO HOW DOES THIS WORK?
rclcpp_components_register_nodes(timer_example PLUGIN "ros2_examples::TimerExample" EXECUTABLE timer_example)

# set(node_plugins "${node_plugins}${PROJECT_NAME}::ServiceServerExample\n")

## --------------------------------------------------------------
## |                       params example                       |
## --------------------------------------------------------------

add_library(params_example SHARED
  src/params_example.cpp
)

# TODO WHY IS THIS HERE?
target_compile_definitions(params_example
  PRIVATE "ros2_examples_BUILDING_DLL")

ament_target_dependencies(params_example
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)

# target_link_libraries(params_example
# )

# TODO HOW DOES THIS WORK?
rclcpp_components_register_nodes(params_example PLUGIN "ros2_examples::ParamsExample" EXECUTABLE params_example)

# set(node_plugins "${node_plugins}${PROJECT_NAME}::ParamsExample\n")

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  params_example
  subscriber_example
  publisher_example
  service_server_example
  service_client_example
  timer_example
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
