#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include "ros2_examples/rate_counter.h"

using namespace std::chrono_literals;

namespace ros2_examples
{

  /* class SubscriberExample //{ */

  class SubscriberExample : public rclcpp::Node
  {
  private:
    using RateCounter = utils::RateCounter;

  public:
    SubscriberExample(rclcpp::NodeOptions options);

  private:
    // | ----------------------- subscribers ---------------------- |

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_fast_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_slow_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_irregular_;

    // | ------------------------- methods ------------------------ |
    
    void callback_fast_subscriber(const std_msgs::msg::String::SharedPtr msg);
    void callback_slow_subscriber(const std_msgs::msg::String::SharedPtr msg);
    void callback_irregular_subscriber(const std_msgs::msg::String::SharedPtr msg);

    // | ------------------ other member variables ---------------- |

    rclcpp::CallbackGroup::SharedPtr callback_group_;

    std::mutex fast_rate_mtx_;
    RateCounter fast_rate_counter_ = RateCounter(get_clock(), 100);

    std::mutex slow_rate_mtx_;
    RateCounter slow_rate_counter_ = RateCounter(get_clock(), 10);

    std::mutex irregular_rate_mtx_;
    RateCounter irregular_rate_counter_ = RateCounter(get_clock(), 10);
  };

  //}

  /* SubscriberExample() //{ */

  SubscriberExample::SubscriberExample(rclcpp::NodeOptions options) : Node("subscriber_example", options)
  {

    RCLCPP_INFO(get_logger(), "[SubscriberExample]: initializing");

    // For good control over how callbacks are executed (in parallel? mutually exclusive?), it is recommended to
    // explicitely create and specify a callback group. The Reentrant type allows all callbacks in the group to run multiple times in parallel,
    // so mutexes are typically necessary to prevent race conditions. The MutuallyExclusive only allows a single callback in the group to run at a single time.
    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    // | ----------------------- SubscriberExample ----------------------- |

    const std::function<void(const std_msgs::msg::String::SharedPtr)> fast_cbk = std::bind(&SubscriberExample::callback_fast_subscriber, this, std::placeholders::_1);
    subscriber_fast_ = create_subscription<std_msgs::msg::String>("~/topic_fast_in", 10, fast_cbk, sub_opt);
    const std::function<void(const std_msgs::msg::String::SharedPtr)> slow_cbk = std::bind(&SubscriberExample::callback_slow_subscriber, this, std::placeholders::_1);
    subscriber_slow_ = create_subscription<std_msgs::msg::String>("~/topic_slow_in", 10, slow_cbk, sub_opt);
    const std::function<void(const std_msgs::msg::String::SharedPtr)> irregular_cbk = std::bind(&SubscriberExample::callback_irregular_subscriber, this, std::placeholders::_1);
    subscriber_irregular_ = create_subscription<std_msgs::msg::String>("~/topic_irregular_in", 10, irregular_cbk, sub_opt);

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(get_logger(), "[SubscriberExample]: initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* callback_subscriber() //{ */

  void SubscriberExample::callback_fast_subscriber(const std_msgs::msg::String::SharedPtr msg)
  {
    std::scoped_lock lck(fast_rate_mtx_);
    const double rate = fast_rate_counter_.update_rate();
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "[SubscriberExample]: receiving string message '" << msg->data << "' at rate " << rate << "Hz.");
  }

  void SubscriberExample::callback_slow_subscriber(const std_msgs::msg::String::SharedPtr msg)
  {
    std::scoped_lock lck(slow_rate_mtx_);
    const double rate = slow_rate_counter_.update_rate();
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "[SubscriberExample]: receiving string message '" << msg->data << "' at rate " << rate << "Hz.");
  }

  void SubscriberExample::callback_irregular_subscriber(const std_msgs::msg::String::SharedPtr msg)
  {
    std::scoped_lock lck(irregular_rate_mtx_);
    const double rate = irregular_rate_counter_.update_rate();
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "[SubscriberExample]: receiving string message '" << msg->data << "' at rate " << rate << "Hz.");
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::SubscriberExample)
