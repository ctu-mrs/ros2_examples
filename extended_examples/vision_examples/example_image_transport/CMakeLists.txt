cmake_minimum_required(VERSION 3.5)
project(vision_examples)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
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
  sensor_msgs
  cv_bridge
  image_transport
)

set(LIBRARIES
  ExampleImageTransport 
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${ros2_examples_INCLUDE_DIRS}
  )

add_library(ExampleImageTransport SHARED
  src/example_image_transport.cpp
)

target_link_libraries(ExampleImageTransport 
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(ExampleImageTransport 
  ${DEPENDENCIES}
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(ExampleImageTransport PLUGIN "vision_examples::ExampleImageTransport" EXECUTABLE ExampleImageTransport)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS
  ${LIBRARIES}
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