#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

  /* class SubscriberExample //{ */

  class SubscriberExample : public rclcpp::Node
  {
  public:
    SubscriberExample(rclcpp::NodeOptions options);

  private:
    // | ----------------------- subscribers ---------------------- |

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

    void callback_subscriber(const std_msgs::msg::String::SharedPtr msg);
  };

  //}

  /* SubscriberExample() //{ */

  SubscriberExample::SubscriberExample(rclcpp::NodeOptions options) : Node("subscriber_example", options)
  {

    RCLCPP_INFO(get_logger(), "[SubscriberExample]: initializing");

    // | ----------------------- SubscriberExample ----------------------- |

    subscriber_ = create_subscription<std_msgs::msg::String>("~/topic_in", 10, std::bind(&SubscriberExample::callback_subscriber, this, std::placeholders::_1));

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(get_logger(), "[SubscriberExample]: initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* callback_subscriber() //{ */

  void SubscriberExample::callback_subscriber(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO_STREAM(get_logger(), "[SubscriberExample]: received string message '" << msg->data << "'");
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::SubscriberExample)
