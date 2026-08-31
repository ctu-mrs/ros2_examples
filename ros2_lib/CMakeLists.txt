cmake_minimum_required(VERSION 3.28)
project(ros2_lib)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

target_include_directories(Ros2Lib_Adder PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(Ros2Lib_Adder
  rclcpp::rclcpp
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

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_libraries(
  ${LIBRARIES}
)

ament_export_include_directories(include)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
