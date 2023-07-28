#include <rclcpp/rclcpp.hpp>
#include <random>

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
    // | -------------------- member variables -------------------- |
    std::default_random_engine reng_;
    std::uniform_real_distribution<> random_time_dist_;

    // | ----------------------- publishers ----------------------- |

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_fast_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_slow_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_irregular_;

    // | ------------------------- timers ------------------------- |

    rclcpp::TimerBase::SharedPtr timer_fast_;
    rclcpp::TimerBase::SharedPtr timer_slow_;
    rclcpp::TimerBase::SharedPtr timer_irregular_;

    // | ------------------------- methods ------------------------ |

    void timer_fast();
    void timer_slow();
    void timer_irregular();
  };

  //}

  /* PublisherExample() //{ */

  PublisherExample::PublisherExample(rclcpp::NodeOptions options) : Node("publisher_example", options)
  {
    RCLCPP_INFO(get_logger(), "[PublisherExample]: initializing");

    random_time_dist_ = std::uniform_real_distribution<>(0.0, 1.0);

    // | ------------------------ publisher ----------------------- |

    publisher_fast_ = create_publisher<std_msgs::msg::String>("~/topic_fast_out", 1);
    publisher_slow_ = create_publisher<std_msgs::msg::String>("~/topic_slow_out", 1);
    publisher_irregular_ = create_publisher<std_msgs::msg::String>("~/topic_irregular_out", 1);

    // | -------------------------- timer ------------------------- |

    timer_fast_ = create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&PublisherExample::timer_fast, this));
    timer_slow_ = create_wall_timer(std::chrono::duration<double>(1.0 / 7.0), std::bind(&PublisherExample::timer_slow, this));
    timer_irregular_ = create_wall_timer(std::chrono::duration<double>(random_time_dist_(reng_)), std::bind(&PublisherExample::timer_irregular, this));

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(get_logger(), "[PublisherExample]: initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* timer_fast() //{ */

  void PublisherExample::timer_fast()
  {
    publisher_fast_->publish(std_msgs::msg::String().set__data("This message should be published at 100Hz."));
  }

  //}

  /* timer_slow() //{ */

  void PublisherExample::timer_slow()
  {
    publisher_slow_->publish(std_msgs::msg::String().set__data("This message should be published at 7Hz."));
  }

  //}

  /* timer_irregular() //{ */

  void PublisherExample::timer_irregular()
  {
    publisher_irregular_->publish(std_msgs::msg::String().set__data("This message should be published at irregular intervals."));
    timer_irregular_ = create_wall_timer(std::chrono::duration<double>(random_time_dist_(reng_)), std::bind(&PublisherExample::timer_irregular, this));
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::PublisherExample)
