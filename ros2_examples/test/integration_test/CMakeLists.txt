get_filename_component(TEST_NAME "${CMAKE_CURRENT_SOURCE_DIR}" NAME)

add_executable(test_${TEST_NAME}
  test.cpp
  )

target_link_libraries(test_${TEST_NAME}
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ros2_lib::Ros2Lib_Adder
  )

install(TARGETS
  test_${TEST_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

add_ros_isolated_launch_test(test.py)
