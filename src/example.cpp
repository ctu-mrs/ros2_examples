#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_uav_example
{

/* class Example //{ */

class Example : public rclcpp::Node {
public:
  Example(rclcpp::NodeOptions options);

private:
  // | ----------------------- publishers ----------------------- |

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // | ----------------------- subscribers ---------------------- |

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  void                                                   callbackSubscriber(const std_msgs::msg::String::SharedPtr msg);

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_;
  void                         callbackTimer();

  // | ----------------------- parameters ----------------------- |

  double      timer_rate_ = 5;
  std::string publish_string_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult callbackParameters(std::vector<rclcpp::Parameter> parameters);

  // | ------------------------ routines ------------------------ |

  void publish(void);
};

//}

/* Example() //{ */

Example::Example(rclcpp::NodeOptions options) : Node("ros2_uav_example", options) {

  RCLCPP_INFO(this->get_logger(), "initializing");

  this->declare_parameter("timer_rate");
  if (!this->get_parameter("timer_rate", timer_rate_)) {
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

  // | ----------------------- parameters ----------------------- |

  param_callback_handle_ = add_on_set_parameters_callback(std::bind(&Example::callbackParameters, this, std::placeholders::_1));

  // | -------------------------- timer ------------------------- |

  timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / timer_rate_), std::bind(&Example::callbackTimer, this));
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTimer() //{ */

void Example::callbackTimer(void) {

  RCLCPP_INFO(this->get_logger(), "timer spinning");

  publish();
}

//}

/* callbackSubscriber() //{ */

void Example::callbackSubscriber(const std_msgs::msg::String::SharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "callbackSubscriber(): received '%s'", msg->data.c_str());
}

//}

/* callbackParameters() //{ */

rcl_interfaces::msg::SetParametersResult Example::callbackParameters(std::vector<rclcpp::Parameter> parameters) {

  RCLCPP_INFO(this->get_logger(), "params updated");

  for (size_t i = 0; i < parameters.size(); i++) {
    std::stringstream ss;
    ss << "{" << parameters[i].get_name() << ", " << parameters[i].value_to_string() << "}";
    RCLCPP_INFO(this->get_logger(), "got parameter: '%s'", ss.str().c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "don't know";

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

}  // namespace ros2_uav_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_uav_example::Example)
