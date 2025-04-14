#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class ClockConsumer //{ */

class ClockConsumer : public rclcpp::Node {
public:
  ClockConsumer(rclcpp::NodeOptions options);

private:
  // | ------------------------- timers ------------------------- |

  rclcpp::CallbackGroup::SharedPtr cb_grp1_;

  rclcpp::TimerBase::SharedPtr timer_1_;

  rclcpp::TimerBase::SharedPtr timer_2_;

  void callback_timer1();

  void callback_timer2();
};

//}

/* ClockConsumer() //{ */

ClockConsumer::ClockConsumer(rclcpp::NodeOptions options) : Node("timer_example", options) {

  RCLCPP_INFO(get_logger(), "[ClockConsumer]: initializing");

  // | -------------------------- timer ------------------------- |

  cb_grp1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // 100 Hz ROS timer
  timer_1_ = this->create_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&ClockConsumer::callback_timer1, this), cb_grp1_);

  timer_1_->cancel();

  // 1 Hz ROS time
  timer_2_ = this->create_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&ClockConsumer::callback_timer2, this), cb_grp1_);

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "[ClockConsumer]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTimer1() //{ */

void ClockConsumer::callback_timer1() {

  RCLCPP_INFO_ONCE(get_logger(), "[Timer 1]: running...");

  // | ---------------------- testing sleep --------------------- |

  RCLCPP_INFO(get_logger(), "[Timer 1]: should be running at 10 Hz simtime");
}

//}

/* callbackTimer2() //{ */

void ClockConsumer::callback_timer2() {

  RCLCPP_INFO_ONCE(get_logger(), "[Timer 2]: running...");

  // | ---------------------- testing sleep --------------------- |

  auto clock = this->get_clock();

  RCLCPP_INFO(get_logger(), "[Timer 2]: now we should sleep for 1.1 seconds");

  clock->sleep_for(std::chrono::duration<double>(1.1s));

  RCLCPP_INFO(get_logger(), "[Timer 2]: sleep finished");

  // | ---------------------- testing rate ---------------------- |

  rclcpp::Rate rate(10.0, clock);

  for (int i = 0; i < 10; i++) {

    RCLCPP_INFO(get_logger(), "[Timer 2]: this info should be comming at 10 Hz simtime");

    rate.sleep();
  }

  // | ------------------------ duration ------------------------ |

  rclcpp::Duration duration(std::chrono::duration<double>(1.1s));

  RCLCPP_INFO(get_logger(), "[Timer 2]: now we should sleep for %.3f seconds", duration.seconds());

  clock->sleep_for(duration);

  RCLCPP_INFO(get_logger(), "[Timer 2]: sleep finished");

  // | -------------------- duration to rate -------------------- |

  timer_2_->cancel();

  timer_1_->reset();
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ClockConsumer)
