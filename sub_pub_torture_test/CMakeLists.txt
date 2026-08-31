cmake_minimum_required(VERSION 3.28)
project(sub_pub_torture_test)

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
  )

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(TARGETS
  rclcpp::rclcpp
  rclcpp_components::component
  ${std_msgs_TARGETS}
)

include_directories(
  include
)

set(LIBRARIES
  subscribers
  publishers
)

## --------------------------------------------------------------
## |                         subscribers                        |
## --------------------------------------------------------------

add_library(subscribers SHARED
  src/subscribers.cpp
)

target_link_libraries(subscribers
  ${TARGETS}
)

rclcpp_components_register_nodes(subscribers PLUGIN "sub_pub_torture_test::Subscribers" EXECUTABLE subscribers)

## --------------------------------------------------------------
## |                         publishers                         |
## --------------------------------------------------------------

add_library(publishers SHARED
  src/publishers.cpp
)

target_link_libraries(publishers
  ${TARGETS}
)

rclcpp_components_register_nodes(publishers PLUGIN "sub_pub_torture_test::Publishers" EXECUTABLE publishers)

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

install(
  TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# this will export the include-related variables for use by other libraries
# this is needed if you wish to include the headers from this pkg into downstream libraries
ament_export_include_directories("include")

ament_package()
