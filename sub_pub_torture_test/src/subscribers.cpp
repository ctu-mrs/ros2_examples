#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <sub_pub_torture_test/rate_counter.h>
#include <sub_pub_torture_test/params.h>

using namespace std::chrono_literals;

namespace sub_pub_torture_test
{

/* class Subscribers //{ */

class Subscribers : public rclcpp::Node {
private:
public:
  Subscribers(rclcpp::NodeOptions options);

private:
  // | ------------------------- params ------------------------- |

  int    _n_subscribers_;
  double _timer_rate_;

  // | ----------------------- subscribers ---------------------- |

  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;

  void callbackSubscriber(const std_msgs::msg::String::SharedPtr msg, const int subscriber_id);

  // | ------------------ other member variables ---------------- |

  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_group_subs_;

  // | ------------------------- timers ------------------------- |

  rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
  rclcpp::TimerBase::SharedPtr     timer_main_;

  void callbackTimerMain();

  std::shared_ptr<utils::RateCounter> timer_rate_counter_;

  rclcpp::Time last_time_timer_;
};

//}

/* Subscribers() //{ */

Subscribers::Subscribers(rclcpp::NodeOptions options) : Node("subscriber_example", options) {

  RCLCPP_INFO(get_logger(), "initializing");

  utils::parse_param("n_subscribers", _n_subscribers_, *this);
  utils::parse_param("timer_rate", _timer_rate_, *this);

  last_time_timer_ = this->get_clock()->now();

  // | ----------------------- Subscribers ----------------------- |

  rclcpp::QoS qos = rclcpp::SystemDefaultsQoS();

  for (int i = 0; i < _n_subscribers_; i++) {

    auto sub_opt = rclcpp::SubscriptionOptions();

    callback_group_subs_.push_back(create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));

    sub_opt.callback_group = callback_group_subs_.at(i);

    RCLCPP_INFO(get_logger(), "initializing subscriber %d", i);

    const std::function<void(const std_msgs::msg::String::SharedPtr)> cbk_ptr = std::bind(&Subscribers::callbackSubscriber, this, std::placeholders::_1, i);

    std::stringstream ss;
    ss << i;

    subscribers_.push_back(create_subscription<std_msgs::msg::String>("/topic_" + ss.str(), qos, cbk_ptr, sub_opt));
  }

  // | -------------------------- timer ------------------------- |

  /* assign each timer callback to a different group so they are run in parallel */
  callback_group_timer_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  timer_main_ = create_timer(std::chrono::duration<double>(1.0 / _timer_rate_), std::bind(&Subscribers::callbackTimerMain, this), callback_group_timer_);

  timer_rate_counter_ = std::make_shared<utils::RateCounter>(this->get_clock());

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackSubscriber() //{ */

void Subscribers::callbackSubscriber([[maybe_unused]] const std_msgs::msg::String::SharedPtr msg, [[maybe_unused]] const int subscriber_id) {
}

//}

/* callbackTimerMain() //{ */

void Subscribers::callbackTimerMain() {

  double rate = timer_rate_counter_->update_rate();

  double dt = (this->get_clock()->now() - last_time_timer_).seconds();

  last_time_timer_ = this->get_clock()->now();

  RCLCPP_INFO(get_logger(), "timer rate %d, dt %.3f", int(round(rate)), dt);
}

//}

}  // namespace sub_pub_torture_test

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(sub_pub_torture_test::Subscribers)
