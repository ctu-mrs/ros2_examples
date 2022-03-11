#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

  /* class PublisherExample //{ */

  class PublisherExample : public rclcpp::Node
  {
  public:
    PublisherExample(rclcpp::NodeOptions options);

  private:
    // | ----------------------- publishers ----------------------- |

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    // | ------------------------- timers ------------------------- |

    rclcpp::TimerBase::SharedPtr timer_main_;

    void timer_main();
  };

  //}

  /* PublisherExample() //{ */

  PublisherExample::PublisherExample(rclcpp::NodeOptions options) : Node("publisher_example", options)
  {
    RCLCPP_INFO(get_logger(), "[PublisherExample]: initializing");

    // | ------------------------ publisher ----------------------- |

    publisher_ = create_publisher<std_msgs::msg::String>("~/topic_out", 1);

    // | -------------------------- timer ------------------------- |

    timer_main_ = create_wall_timer(std::chrono::duration<double>(1.0 / 2.0), std::bind(&PublisherExample::timer_main, this));

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(get_logger(), "[PublisherExample]: initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* timer_main() //{ */

  void PublisherExample::timer_main()
  {
    RCLCPP_INFO(get_logger(), "[PublisherExample]: publishing");
    publisher_->publish(std_msgs::msg::String().set__data("ahoj!"));
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::PublisherExample)
