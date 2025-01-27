#ifndef INTEGRATION_TEST_H
#define INTEGRATION_TEST_H

#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include <thread>

namespace ros2_lib
{

class IntegrationTest {

public:
  IntegrationTest();

  rclcpp::Node::SharedPtr                              node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_result_;

  void reportTesResult(const bool result);

  std::thread main_thread_;

  void spin();

  void join();
};

// constructor
IntegrationTest::IntegrationTest() {

  node_     = rclcpp::Node::make_shared("test");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_node(node_);

  publisher_result_ = node_->create_publisher<std_msgs::msg::Bool>("/test_result", 100);

  main_thread_ = std::thread(&IntegrationTest::spin, this);
}

void IntegrationTest::reportTesResult(const bool result) {

  printf("[IntegrationTest]: publishing result %s", result ? "SUCCESS" : "FAILED");

  std_msgs::msg::Bool result_msg;
  result_msg.data = result;

  publisher_result_->publish(result_msg);
}

void IntegrationTest::spin() {

  printf("[IntegrationTest]: spinning");

  executor_->spin();
}

void IntegrationTest::join() {

  printf("[IntegrationTest]: joined");

  main_thread_.join();
}

}  // namespace ros2_lib

#endif  // INTEGRATION_TEST_H
