#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class ServiceServerExample //{ */

class ServiceServerExample : public rclcpp::Node {
public:
  ServiceServerExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | --------------------- service servers -------------------- |

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_;

  void callbackSetBool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
};

//}

/* ServiceServerExample() //{ */

ServiceServerExample::ServiceServerExample(rclcpp::NodeOptions options) : Node("service_server_example", options) {

  RCLCPP_INFO(this->get_logger(), "[ServiceServerExample]: initializing");

  // | --------------------- service server --------------------- |

  service_server_ =
      this->create_service<std_srvs::srv::SetBool>("~/set_bool_in", std::bind(&ServiceServerExample::callbackSetBool, this, std::placeholders::_1, std::placeholders::_2));

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[ServiceServerExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackSetBool() //{ */

void ServiceServerExample::callbackSetBool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[ServiceServerExample]: received service call: %s", request->data ? "TRUE" : "FALSE");

  response->message = "succeeded";
  response->success = true;
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceServerExample)
