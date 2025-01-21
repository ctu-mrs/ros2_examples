#include <rclcpp/rclcpp.hpp>

#include <random>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class PublisherExample //{ */

class PublisherExample : public rclcpp::Node {
public:
  PublisherExample(const rclcpp::NodeOptions options);

private:
  // | -------------------- member variables -------------------- |

  std::default_random_engine       reng_;
  std::uniform_real_distribution<> random_time_dist_;

  // | ----------------------- publishers ----------------------- |

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_fast_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_slow_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_irregular_;

  // | ------------------------- timers ------------------------- |

  /* assigning each timer to a separate callback group ensures they are run in parallel */
  rclcpp::CallbackGroup::SharedPtr callback_group_fast_;
  rclcpp::TimerBase::SharedPtr     timer_fast_;

  rclcpp::CallbackGroup::SharedPtr callback_group_slow_;
  rclcpp::TimerBase::SharedPtr     timer_slow_;

  rclcpp::CallbackGroup::SharedPtr callback_group_irregular_;
  rclcpp::TimerBase::SharedPtr     timer_irregular_;

  // | ------------------------- methods ------------------------ |

  void callback_timer_fast();
  void callback_timer_slow();
  void callback_timer_irregular();
};

//}

/* PublisherExample() //{ */

PublisherExample::PublisherExample(rclcpp::NodeOptions options) : Node("publisher_example", options) {

  RCLCPP_INFO(get_logger(), "initializing");

  random_time_dist_ = std::uniform_real_distribution<>(0.0, 1.0);

  // | ------------------------ publisher ----------------------- |

  publisher_fast_      = create_publisher<std_msgs::msg::String>("~/topic_fast_out", 1);
  publisher_slow_      = create_publisher<std_msgs::msg::String>("~/topic_slow_out", 1);
  publisher_irregular_ = create_publisher<std_msgs::msg::String>("~/topic_irregular_out", 1);

  // | -------------------------- timer ------------------------- |

  /* assign each timer callback to a different group so they are run in parallel */
  callback_group_fast_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_fast_ = create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&PublisherExample::callback_timer_fast, this), callback_group_fast_);

  callback_group_slow_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_slow_ = create_wall_timer(std::chrono::duration<double>(1.0 / 2.0), std::bind(&PublisherExample::callback_timer_slow, this), callback_group_slow_);

  callback_group_irregular_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_irregular_ = create_wall_timer(std::chrono::duration<double>(random_time_dist_(reng_)), std::bind(&PublisherExample::callback_timer_irregular, this),
                                       callback_group_irregular_);

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callback_timer_fast() //{ */

void PublisherExample::callback_timer_fast() {

  publisher_fast_->publish(std_msgs::msg::String().set__data("This message should be published at 100Hz."));
}

//}

/* callback_timer_slow() //{ */

void PublisherExample::callback_timer_slow() {

  publisher_slow_->publish(std_msgs::msg::String().set__data("This message should be published at 2Hz."));
}

//}

/* callback_timer_irregular() //{ */

void PublisherExample::callback_timer_irregular() {

  publisher_irregular_->publish(std_msgs::msg::String().set__data("This message should be published at irregular intervals."));

  /* reset the rate of this timer to a random number */
  /* the average rate for this timer will be 2.5Hz as the average period is 0.5s */
  timer_irregular_ = create_wall_timer(std::chrono::duration<double>(random_time_dist_(reng_)), std::bind(&PublisherExample::callback_timer_irregular, this));
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::PublisherExample)
