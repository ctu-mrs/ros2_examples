#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_uav_example
{

/* class Example //{ */

class Example : public rclcpp::Node {
public:
  Example(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ----------------------- publishers ----------------------- |

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void publish(void);

  // | ----------------------- subscribers ---------------------- |

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  void callbackSubscriber(const std_msgs::msg::String::SharedPtr msg);

  // | --------------------- service servers -------------------- |

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_server_;

  void callbackSetBool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // | --------------------- service clients -------------------- |

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;

  void callService(void);

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_;

  void callbackTimer();

  rclcpp::TimerBase::SharedPtr timer_blocking_;

  void callbackTimerBlocking();

  // | ----------------------- parameters ----------------------- |

  double      timer_rate_ = 5;
  std::string publish_string_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult callbackParameters(std::vector<rclcpp::Parameter> parameters);
};

//}

/* Example() //{ */

Example::Example(rclcpp::NodeOptions options) : Node("ros2_uav_example", options) {

  RCLCPP_INFO(this->get_logger(), "initializing");

  this->declare_parameter("timer/rate");
  if (!this->get_parameter("timer/rate", timer_rate_)) {
    RCLCPP_ERROR(this->get_logger(), "could not load param");
  }

  this->declare_parameter("publish_string");
  if (!this->get_parameter("publish_string", publish_string_)) {
    RCLCPP_ERROR(this->get_logger(), "could not load param");
  }

  // parameter from the launch file
  this->declare_parameter("uav_type");
  std::string uav_type;
  if (!this->get_parameter("uav_type", uav_type)) {
    RCLCPP_ERROR(this->get_logger(), "could not load param");
  }
  RCLCPP_ERROR(this->get_logger(), "UAV_TYPE: %s", uav_type.c_str());

  // | ------------------------ publisher ----------------------- |

  publisher_ = this->create_publisher<std_msgs::msg::String>("~/topic_out", 1);

  // | ----------------------- subscriber ----------------------- |

  subscriber_ = this->create_subscription<std_msgs::msg::String>("~/topic_in", 10, std::bind(&Example::callbackSubscriber, this, std::placeholders::_1));

  // | --------------------- service server --------------------- |

  service_server_ =
      this->create_service<std_srvs::srv::SetBool>("~/set_bool_in", std::bind(&Example::callbackSetBool, this, std::placeholders::_1, std::placeholders::_2));

  // | --------------------- service client --------------------- |

  service_client_ = this->create_client<std_srvs::srv::SetBool>("~/set_bool_out");

  // | ----------------------- parameters ----------------------- |

  param_callback_handle_ = add_on_set_parameters_callback(std::bind(&Example::callbackParameters, this, std::placeholders::_1));

  // | -------------------------- timer ------------------------- |

  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&Example::callbackTimer, this));

  // will this timer block other callbacks?
  /* timer_blocking_ = this->create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&Example::callbackTimerBlocking, this)); */

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[Example]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTimer() //{ */

void Example::callbackTimer(void) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Timer spinning");
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Timer spinning, throttled");

  publish();

  callService();
}

//}

/* callbackTimerBlocking() //{ */

void Example::callbackTimerBlocking(void) {

  if (!is_initialized_) {
    return;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(1000));
}

//}

/* callbackSubscriber() //{ */

void Example::callbackSubscriber(const std_msgs::msg::String::SharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "callbackSubscriber(): received '%s'", msg->data.c_str());
}

//}

/* callbackSetBool() //{ */

void Example::callbackSetBool(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[Example]: received service call: %s", request->data ? "TRUE" : "FALSE");

  response->message = "succeeded";
  response->success = true;
}

//}

/* callbackParameters() //{ */

rcl_interfaces::msg::SetParametersResult Example::callbackParameters(std::vector<rclcpp::Parameter> parameters) {

  if (!is_initialized_) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason     = "node is not initialized";
    return result;
  }

  RCLCPP_INFO(this->get_logger(), "params updated");

  for (size_t i = 0; i < parameters.size(); i++) {
    std::stringstream ss;
    ss << "{" << parameters[i].get_name() << ", " << parameters[i].value_to_string() << "}";
    RCLCPP_INFO(this->get_logger(), "got parameter: '%s'", ss.str().c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason     = "don't know";

  return result;
}

//}

// | ------------------------ routines ------------------------ |

/* publish() //{ */

void Example::publish(void) {

  RCLCPP_INFO(this->get_logger(), "publishing");

  std_msgs::msg::String string_out;
  string_out.data = publish_string_;
  publisher_->publish(string_out);
}

//}

/* callService() //{ */

void Example::callService(void) {

  RCLCPP_INFO(this->get_logger(), "[Example]: calling service");

  auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  {
    // TODO THIS IS HOW YOU ARE SUPPOSED TO WAIT FOR THE SERVICE TO BECOME READY
    // TODO IT MESSES UP SOMETHING WITH THE GRANULARITY OF PUBLISHING
    // TODO THIS IS FROM AN EXAMPLE

    /* while (!service_client_->wait_for_service(1s)) { */
    /*   if (!rclcpp::ok()) { */
    /*     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting."); */
    /*   } */
    /*   RCLCPP_INFO(this->get_logger(), "service not available, waiting again..."); */
    /*   break; */
    /* } */
  }

  // define a callback for the service response
  using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "[Example]: service result received");
  };

  // asynchronous call
  auto result = service_client_->async_send_request(request, response_received_callback);  // with a callback
  /* auto result = service_client_->async_send_request(request); // without a callback */

  {
      // TODO ONE WAY OF WAITING FOR THE RESULT IS TO CHECK THE std::future
      // TODO THIS BLOCKS THE WHOLE NODE FROM GETTING CALLBACKS, THEREFORE, NOTHING WORKS

      /* std::future_status status; */
      /* do { */
      /*   status = result.wait_for(std::chrono::seconds(1)); */
      /*   if (status == std::future_status::deferred) { */
      /*     RCLCPP_INFO(this->get_logger(), "[Example]: deferred"); */
      /*   } else if (status == std::future_status::timeout) { */
      /*     RCLCPP_INFO(this->get_logger(), "[Example]: timeout"); */
      /*   } else if (status == std::future_status::ready) { */
      /*     RCLCPP_INFO(this->get_logger(), "[Example]: ready"); */
      /*   } else { */
      /*     RCLCPP_INFO(this->get_logger(), "[Example]: something else"); */
      /*   } */
      /* } while (status != std::future_status::ready); */
  }

  {
    // TODO ANOTHER WAY HOT TO WAIT FOR THE RESULT IS TO "spin_until_future_complete"
    // TODO THIS IS HOW YOU WOULD WAIT FOR THE RESPONSE IN A NORMAL NODE
    // TODO BUT WE DON'T HAVE THE "NODE"

    /* rclcpp::spin_until_future_complete(node, result); */
  }
}

//}

}  // namespace ros2_uav_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_uav_example::Example)
