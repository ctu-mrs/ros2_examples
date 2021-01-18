#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class PublisherExample //{ */

class PublisherExample : public rclcpp::Node {
public:
  PublisherExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ----------------------- publishers ----------------------- |

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  void publish(void);

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;

  void timerMain();
};

//}

/* PublisherExample() //{ */

PublisherExample::PublisherExample(rclcpp::NodeOptions options) : Node("publisher_example", options) {

  RCLCPP_INFO(this->get_logger(), "[PublisherExample]: initializing");

  // | ------------------------ publisher ----------------------- |

  publisher_ = this->create_publisher<std_msgs::msg::String>("~/topic_out", 1);

  // | -------------------------- timer ------------------------- |

  timer_main_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&PublisherExample::timerMain, this));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[PublisherExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* timerMain() //{ */

void PublisherExample::timerMain(void) {

  if (!is_initialized_) {
    return;
  }

  publish();
}

//}

// | ------------------------ routines ------------------------ |

/* publish() //{ */

void PublisherExample::publish(void) {

  RCLCPP_INFO(this->get_logger(), "[PublisherExample]: publishing");

  std_msgs::msg::String string_out;
  string_out.data = "test string";
  publisher_->publish(string_out);
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::PublisherExample)
