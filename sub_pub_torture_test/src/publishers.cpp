#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <sub_pub_torture_test/rate_counter.h>
#include <sub_pub_torture_test/params.h>

using namespace std::chrono_literals;

namespace sub_pub_torture_test
{

/* class Publishers //{ */

class Publishers : public rclcpp::Node {
public:
  Publishers(const rclcpp::NodeOptions options);

private:
  // | -------------------- member variables -------------------- |

  int    _n_publishers_;
  double _timer_rate_;

  // | ----------------------- publishers ----------------------- |

  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;

  // | ------------------------- timers ------------------------- |

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr     timer_main_;

  std::shared_ptr<utils::RateCounter> timer_rate_counter_;

  // | ------------------------- methods ------------------------ |

  void callbackTimerMain();
};

//}

/* Publishers() //{ */

Publishers::Publishers(rclcpp::NodeOptions options) : Node("publisher_example", options) {

  RCLCPP_INFO(get_logger(), "initializing");

  utils::parse_param("n_publishers", _n_publishers_, *this);
  utils::parse_param("timer_rate", _timer_rate_, *this);

  // | ------------------------ publisher ----------------------- |

  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  for (int i = 0; i < _n_publishers_; i++) {

    RCLCPP_INFO(get_logger(), "initializing publisher %d", i);

    std::stringstream ss;
    ss << i;

    publishers_.push_back(create_publisher<std_msgs::msg::String>("/topic_" + ss.str(), qos));
  }

  // | -------------------------- timer ------------------------- |

  callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_main_ = create_wall_timer(std::chrono::duration<double>(1.0 / _timer_rate_), std::bind(&Publishers::callbackTimerMain, this), callback_group_);

  timer_rate_counter_ = std::make_shared<utils::RateCounter>(this->get_clock());

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTimerMain() //{ */

void Publishers::callbackTimerMain() {

  double rate = timer_rate_counter_->update_rate();

  RCLCPP_INFO(get_logger(), "timer rate %d", int(round(rate)));

  RCLCPP_INFO_ONCE(get_logger(), "timer_main_ running");

  for (int i = 0; i < _n_publishers_; i++) {

    publishers_[i]->publish(std_msgs::msg::String().set__data("some data"));
  }
}

//}

}  // namespace sub_pub_torture_test

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sub_pub_torture_test::Publishers)
