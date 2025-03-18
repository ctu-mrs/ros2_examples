cmake_minimum_required(VERSION 3.5)
project(vision_examples)

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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS highgui imgcodecs imgproc videoio)

add_library(example_image_transport SHARED
  src/example_image_transport.cpp
)

target_link_libraries(example_image_transport 
  ${OpenCV_LIBRARIES}
)

ament_target_dependencies(example_image_transport 
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  cv_bridge
)

# each component (nodelet) needs to be registered to make it discoverable at runtime
rclcpp_components_register_nodes(example_image_transport PLUGIN "vision_examples::ExampleImageTransport" EXECUTABLE example_image_transport)

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS
  example_image_transport
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