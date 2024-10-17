#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

  /* class ServiceServerExample //{ */

  class ServiceServerExample : public rclcpp::Node
  {
  public:
    ServiceServerExample(const rclcpp::NodeOptions &options);

  private:
    // | --------------------- service servers -------------------- |

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_;

    void callback_set_bool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);
  };

  //}

  /* ServiceServerExample() //{ */

  ServiceServerExample::ServiceServerExample(const rclcpp::NodeOptions &options) : Node("service_server_example", options)
  {

    RCLCPP_INFO(get_logger(), "initializing");

    // | --------------------- service server --------------------- |

    service_server_ = create_service<std_srvs::srv::SetBool>(
        "~/set_bool_in", std::bind(&ServiceServerExample::callback_set_bool, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* callback_set_bool() //{ */

  void ServiceServerExample::callback_set_bool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                                               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    RCLCPP_INFO_STREAM(get_logger(), "Received service call: " << request->data ? "TRUE" : "FALSE");

    response->message = "succeeded";
    response->success = true;
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceServerExample)
