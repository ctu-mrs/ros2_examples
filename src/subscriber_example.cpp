#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class SubscriberExample //{ */

class SubscriberExample : public rclcpp::Node {
public:
  SubscriberExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ----------------------- subscribers ---------------------- |

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  void callbackSubscriber(const std_msgs::msg::String::SharedPtr msg);
};

//}

/* SubscriberExample() //{ */

SubscriberExample::SubscriberExample(rclcpp::NodeOptions options) : Node("subscriber_example", options) {

  RCLCPP_INFO(this->get_logger(), "[SubscriberExample]: initializing");

  // | ----------------------- SubscriberExample ----------------------- |

  subscriber_ = this->create_subscription<std_msgs::msg::String>("~/topic_in", 10, std::bind(&SubscriberExample::callbackSubscriber, this, std::placeholders::_1));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[SubscriberExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackSubscriber() //{ */

void SubscriberExample::callbackSubscriber(const std_msgs::msg::String::SharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[SubscriberExample]: receiving '%s'",  msg->data.c_str());
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::SubscriberExample)
