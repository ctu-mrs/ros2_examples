#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include <ros2_lib/integration_test.h>

#include <std_msgs/msg/int64.hpp>

using namespace std::chrono_literals;

class Test : public ros2_lib::IntegrationTest {

public:
  Test() : ros2_lib::IntegrationTest() {
  }

  bool testMethod1(void);
};

/* testPublish() //{ */

bool Test::testMethod1(void) {

  RCLCPP_INFO(node_->get_logger(), "sleeping for 10 s");

  rclcpp::sleep_for(std::chrono::milliseconds(10000));

  RCLCPP_INFO(node_->get_logger(), "stopping the tests");

  return true;
}

//}

int main(int argc, char* argv[]) {

  rclcpp::init(argc, argv);

  bool test_result = true;

  Test test;

  test_result &= test.testMethod1();

  test.reportTesResult(test_result);

  test.join();
}
