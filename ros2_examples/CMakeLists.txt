cmake_minimum_required(VERSION 3.5)
project(ros2_examples)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
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
  tf2_ros
  geometry_msgs
  ros2_lib
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  params_example
  subscriber_example
  publisher_example
  service_server_example
  service_client_example
  timer_example
  tf2_broadcaster_example
  tf2_listener_example
  clock_consumer
  )

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                     subscriber example                     |
## --------------------------------------------------------------

add_library(subscriber_example SHARED
  src/subscriber_example.cpp
)

# ament manages cmake related variables and dependency search (similar to catkin in ROS1)
ament_target_dependencies(subscriber_example
  rclcpp
  rclcpp_components
  std_msgs
  ros2_lib
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(subscriber_example PLUGIN "ros2_examples::SubscriberExample" EXECUTABLE subscriber_example)

## --------------------------------------------------------------
## |                      publisher example                     |
## --------------------------------------------------------------

add_library(publisher_example SHARED
  src/publisher_example.cpp
)

ament_target_dependencies(publisher_example
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(publisher_example PLUGIN "ros2_examples::PublisherExample")

## --------------------------------------------------------------
## |                   service server example                   |
## --------------------------------------------------------------

add_library(service_server_example SHARED
  src/service_server_example.cpp
)

ament_target_dependencies(service_server_example
  rclcpp
  rclcpp_components
  std_srvs
)

rclcpp_components_register_nodes(service_server_example PLUGIN "ros2_examples::ServiceServerExample")

## --------------------------------------------------------------
## |                   service client example                   |
## --------------------------------------------------------------

add_library(service_client_example SHARED
  src/service_client_example.cpp
)

ament_target_dependencies(service_client_example
  rclcpp
  rclcpp_components
  std_srvs
)

rclcpp_components_register_nodes(service_client_example PLUGIN "ros2_examples::ServiceClientExample")

## --------------------------------------------------------------
## |                        timer example                       |
## --------------------------------------------------------------

add_library(timer_example SHARED
  src/timer_example.cpp
)

ament_target_dependencies(timer_example
  rclcpp
  rclcpp_components
  std_msgs
)

rclcpp_components_register_nodes(timer_example PLUGIN "ros2_examples::TimerExample" EXECUTABLE timer_example)

## --------------------------------------------------------------
## |                       params example                       |
## --------------------------------------------------------------

add_library(params_example SHARED
  src/params_example.cpp
)

# # TODO WHY IS THIS HERE?
# target_compile_definitions(params_example
#   PRIVATE "ros2_examples_BUILDING_DLL")

ament_target_dependencies(params_example
  rclcpp
  rclcpp_components
  std_msgs
  std_srvs
)

rclcpp_components_register_nodes(params_example PLUGIN "ros2_examples::ParamsExample" EXECUTABLE params_example)

## --------------------------------------------------------------
## |                    tf2_broadcaster_example                 |
## --------------------------------------------------------------

add_library(tf2_broadcaster_example SHARED
  src/tf2_broadcaster_example.cpp
)

ament_target_dependencies(tf2_broadcaster_example
  rclcpp
  rclcpp_components
  tf2_ros
  geometry_msgs
)

rclcpp_components_register_nodes(tf2_broadcaster_example PLUGIN "ros2_examples::Tf2BroadcasterExample" EXECUTABLE tf2_broadcaster_example)

## --------------------------------------------------------------
## |                    tf2_broadcaster_example                 |
## --------------------------------------------------------------

add_library(tf2_listener_example SHARED
  src/tf2_listener_example.cpp
)

ament_target_dependencies(tf2_listener_example
  rclcpp
  rclcpp_components
  tf2_ros
  geometry_msgs
)

rclcpp_components_register_nodes(tf2_listener_example PLUGIN "ros2_examples::Tf2ListenerExample" EXECUTABLE tf2_listener_example)

## --------------------------------------------------------------
## |                      clock management                      |
## --------------------------------------------------------------

add_library(clock_consumer SHARED
  src/clock_consumer.cpp
)

ament_target_dependencies(clock_consumer
  rclcpp
  rclcpp_components
)

rclcpp_components_register_nodes(clock_consumer PLUGIN "ros2_examples::ClockConsumer" EXECUTABLE clock_consumer)

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(BUILD_TESTING)

  message(WARNING "Testing enabled.")

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS ${LIBRARIES}
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

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
