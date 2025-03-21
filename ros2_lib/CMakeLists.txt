cmake_minimum_required(VERSION 3.12)
project(ros2_lib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Wpedantic")

set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  Eigen3
)


foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

include_directories(
  include
  ${colcon_INCLUDE_DIRS}
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

set(LIBRARIES
  Ros2Lib_Adder
  )

## Geometry

add_library(Ros2Lib_Adder SHARED
  src/adder.cpp
  )

ament_target_dependencies(Ros2Lib_Adder
  rclcpp
  ${Eigen_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Testing                          |
## --------------------------------------------------------------

if(BUILD_TESTING)

  add_subdirectory(test)

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
)

install(
  TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include
  DESTINATION .
)

ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
