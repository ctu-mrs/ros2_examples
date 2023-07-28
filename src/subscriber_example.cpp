#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

  /* class SubscriberExample //{ */

  class SubscriberExample : public rclcpp::Node
  {
  private:

    /* helper class RateMonitor //{ */
    
    class RateMonitor
    {
      public:
        RateMonitor(const unsigned long window, const rclcpp::Time& first_t)
          : window(window)
        {
          reset(first_t);
        }
    
        void reset(const rclcpp::Time& now)
        {
          prev_t = now;
          n_samples = 0;
          mean = 0;
        }
    
        bool update(const rclcpp::Time& next_t)
        {
          bool before_reset = false;
          if (n_samples == window)
          {
            reset(next_t);
            return false;
          }
          else if (n_samples == window - 1)
            before_reset = true;

          // Welford's algorithm according to https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
          n_samples += 1;
          const double dt = (next_t - prev_t).seconds();
          prev_t = next_t;
          const double delta = dt - mean;
          mean += delta / static_cast<double>(n_samples);
          return before_reset;
        }
    
        double get_mean()
        {
          if (n_samples < 2)
            return std::numeric_limits<double>::quiet_NaN();
          else
            // we want the rate, not the period, so invert it
            return 1.0 / mean;
        }
    
      private:
        const unsigned long window;
        rclcpp::Time prev_t;
        unsigned long n_samples;
        double mean;
    };
    
    //}

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
    RateMonitor fast_rate_monitor_ = RateMonitor(100, get_clock()->now());

    std::mutex slow_rate_mtx_;
    RateMonitor slow_rate_monitor_ = RateMonitor(10, get_clock()->now());

    std::mutex irregular_rate_mtx_;
    RateMonitor irregular_rate_monitor_ = RateMonitor(10, get_clock()->now());
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
    const bool monitor_done = fast_rate_monitor_.update(get_clock()->now());
    if (monitor_done)
      RCLCPP_INFO_STREAM(get_logger(), "[SubscriberExample]: receiving string message '" << msg->data << "' at rate " << fast_rate_monitor_.get_mean() << "Hz.");
  }

  void SubscriberExample::callback_slow_subscriber(const std_msgs::msg::String::SharedPtr msg)
  {
    std::scoped_lock lck(slow_rate_mtx_);
    const bool monitor_done = slow_rate_monitor_.update(get_clock()->now());
    if (monitor_done)
      RCLCPP_INFO_STREAM(get_logger(), "[SubscriberExample]: receiving string message '" << msg->data << "' at rate " << slow_rate_monitor_.get_mean() << "Hz.");
  }

  void SubscriberExample::callback_irregular_subscriber(const std_msgs::msg::String::SharedPtr msg)
  {
    std::scoped_lock lck(irregular_rate_mtx_);
    const bool monitor_done = irregular_rate_monitor_.update(get_clock()->now());
    if (monitor_done)
      RCLCPP_INFO_STREAM(get_logger(), "[SubscriberExample]: receiving string message '" << msg->data << "' at rate " << irregular_rate_monitor_.get_mean() << "Hz.");
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::SubscriberExample)
