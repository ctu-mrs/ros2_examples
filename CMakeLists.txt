cmake_minimum_required(VERSION 3.5)
project(ros2_examples)

# set the correct standards
set(CMAKE_C_STANDARD 99)
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
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_ros REQUIRED)

include_directories(
  include
)

## --------------------------------------------------------------
## |                     subscriber example                     |
## --------------------------------------------------------------

# add_library(subscriber_example SHARED
#   src/subscriber_example.cpp
# )

# # TODO WHY IS THIS HERE?
# target_compile_definitions(subscriber_example
#   PRIVATE "ros2_examples_BUILDING_DLL")

# ament_target_dependencies(subscriber_example
#   rclcpp
#   rclcpp_components
#   std_msgs
# )

# # TODO HOW DOES THIS WORK?
# rclcpp_components_register_nodes(subscriber_example PLUGIN "ros2_examples::SubscriberExample" EXECUTABLE subscriber_example)

## --------------------------------------------------------------
## |                      publisher example                     |
## --------------------------------------------------------------

# add_library(publisher_example SHARED
#   src/publisher_example.cpp
# )

# # TODO WHY IS THIS HERE?
# target_compile_definitions(publisher_example
#   PRIVATE "ros2_examples_BUILDING_DLL")

# ament_target_dependencies(publisher_example
#   rclcpp
#   rclcpp_components
#   std_msgs
# )

# # TODO HOW DOES THIS WORK?
# rclcpp_components_register_nodes(publisher_example PLUGIN "ros2_examples::PublisherExample" EXECUTABLE publisher_example)

## --------------------------------------------------------------
## |                   service server example                   |
## --------------------------------------------------------------

add_library(service_server_example SHARED
  src/service_server_example.cpp
)

# TODO WHY IS THIS HERE?
# target_compile_definitions(service_server_example
#   PRIVATE "ros2_examples_BUILDING_DLL")

# ament manages cmake related variables and dependecy search (similar to catkin in ROS1)
ament_target_dependencies(service_server_example
  rclcpp
  rclcpp_components
  std_srvs
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(service_server_example PLUGIN "ros2_examples::ServiceServerExample")

## --------------------------------------------------------------
## |                   service client example                   |
## --------------------------------------------------------------

add_library(service_client_example SHARED
  src/service_client_example.cpp
)

# TODO WHY IS THIS HERE?
# target_compile_definitions(service_client_example
#   PRIVATE "ros2_examples_BUILDING_DLL")

# ament manages cmake related variables and dependecy search (similar to catkin in ROS1)
ament_target_dependencies(service_client_example
  rclcpp
  rclcpp_components
  std_srvs
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(service_client_example PLUGIN "ros2_examples::ServiceClientExample")

## --------------------------------------------------------------
## |                        timer example                       |
## --------------------------------------------------------------

# add_library(timer_example SHARED
#   src/timer_example.cpp
# )

# # TODO WHY IS THIS HERE?
# target_compile_definitions(timer_example
#   PRIVATE "ros2_examples_BUILDING_DLL")

# ament_target_dependencies(timer_example
#   rclcpp
#   rclcpp_components
#   std_srvs
# )

# # TODO HOW DOES THIS WORK?
# rclcpp_components_register_nodes(timer_example PLUGIN "ros2_examples::TimerExample" EXECUTABLE timer_example)

## --------------------------------------------------------------
## |                       params example                       |
## --------------------------------------------------------------

# add_library(params_example SHARED
#   src/params_example.cpp
# )

# # TODO WHY IS THIS HERE?
# target_compile_definitions(params_example
#   PRIVATE "ros2_examples_BUILDING_DLL")

# ament_target_dependencies(params_example
#   rclcpp
#   rclcpp_components
#   std_msgs
#   std_srvs
# )

# makes a component executable
# rclcpp_components_register_nodes(lib_name PLUGIN "namespace::class_name" EXECUTABLE executable_name)
# # lib_name = add_library(lib_name ...
# # executable_name => ros2 run package_name executable_name
# rclcpp_components_register_nodes(params_example PLUGIN "ros2_examples::ParamsExample" EXECUTABLE params_example)

## --------------------------------------------------------------
## |                    tf2_broadcaster_example                 |
## --------------------------------------------------------------

# add_library(tf2_broadcaster_example SHARED
#   src/tf2_broadcaster_example.cpp
# )

# # TODO WHY IS THIS HERE?
# target_compile_definitions(tf2_broadcaster_example
#   PRIVATE "tf2_broadcaster_example_BUILDING_DLL")

# ament_target_dependencies(tf2_broadcaster_example
#   rclcpp
#   rclcpp_components
#   tf2_ros
#   geometry_msgs
# )

# # TODO HOW DOES THIS WORK?
# rclcpp_components_register_nodes(tf2_broadcaster_example PLUGIN "ros2_examples::Tf2BroadcasterExample" EXECUTABLE tf2_broadcaster_example)

## --------------------------------------------------------------
## |                    tf2_broadcaster_example                 |
## --------------------------------------------------------------

# add_library(tf2_listener_example SHARED
#   src/tf2_listener_example.cpp
# )

# # TODO WHY IS THIS HERE?
# target_compile_definitions(tf2_listener_example
#   PRIVATE "tf2_listener_example_BUILDING_DLL")

# ament_target_dependencies(tf2_listener_example
#   rclcpp
#   rclcpp_components
#   tf2_ros
#   geometry_msgs
# )

# # TODO HOW DOES THIS WORK?
# rclcpp_components_register_nodes(tf2_listener_example PLUGIN "ros2_examples::Tf2ListenerExample" EXECUTABLE tf2_listener_example)

## --------------------------------------------------------------
## |                    bank of unused stuff                    |
## --------------------------------------------------------------

# target_link_libraries(timer_example
# )

# set(node_plugins "${node_plugins}${PROJECT_NAME}::ServiceServerExample\n")

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(TARGETS
  # params_example
  # subscriber_example
  # publisher_example
  service_server_example
  service_client_example
  # timer_example
  # tf2_broadcaster_example
  # tf2_listener_example
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
