#include "std_msgs/msg/float64.hpp"
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float64.hpp>

#include <ros2_examples/rate_counter.h>
#include <ros2_examples/params.h>

using namespace std::chrono_literals;

namespace ros2_examples
{

using namespace utils;

/* class TimerExample //{ */

class TimerExample : public rclcpp::Node {
public:
  TimerExample(rclcpp::NodeOptions options);

private:
  // | ------------------------- timers ------------------------- |

  rclcpp::CallbackGroup::SharedPtr cb_grp1_;
  rclcpp::CallbackGroup::SharedPtr cb_grp2_;
  rclcpp::CallbackGroup::SharedPtr cb_grp3_;
  rclcpp::CallbackGroup::SharedPtr cb_grp4_;

  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::TimerBase::SharedPtr timer_2_;
  rclcpp::TimerBase::SharedPtr timer_3_;
  rclcpp::TimerBase::SharedPtr timer_4_;

  void callback_timer1();
  void callback_timer2();
  void callback_timer3();
  void callback_timer4();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_timer_1_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_timer_2_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_timer_3_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_timer_4_;
};

//}

/* TimerExample() //{ */

TimerExample::TimerExample(rclcpp::NodeOptions options) : Node("timer_example", options) {

  RCLCPP_INFO(get_logger(), "[TimerExample]: initializing");

  // | -------------------------- timer ------------------------- |

  bool              loaded_successfully = true;
  const std::string callback_group_type = parse_param2<std::string>("callback_group_type", loaded_successfully, *this);

  if (!loaded_successfully) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  if (callback_group_type == "MutuallyExclusive") {

    // In this case, all callbacks are part of the same MutuallyExclusive group, so only a single callback will be running
    // at one time. This means that one callback that takes a long time will cause the others to stall. You will see that
    // when running this example with the "callback_group_type" parameter set to "MutuallyExclusive".
    cb_grp1_ = cb_grp2_ = cb_grp3_ = cb_grp4_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  } else if (callback_group_type == "Reentrant") {

    // In this case, all callbacks are part of the same Reentrant group, so all any number of callbacks (even of the same type)
    // can run in parallel. No blocking will happen in this case.
    cb_grp1_ = cb_grp2_ = cb_grp3_ = cb_grp4_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  } else if (callback_group_type == "UniqueMutuallyExclusives") {

    // In this case, every callback has its own MutuallyExclusive group, so the different callbacks can run in parallel, but
    // each callback can only run once at a time. This can be useful if you don't want the same callback to execute twice
    // e.g. to avoid having to use mutexes or too much CPU load. But if your callback function takes a too long time, some
    // callbacks may not get executed (see the rate of the 0.5Hz callback executing a 4s workload).
    cb_grp1_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_grp2_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_grp3_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    cb_grp4_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  } else {

    RCLCPP_ERROR_STREAM(get_logger(), "The callback group type must be one of 'MutuallyExclusive', 'Reentrant', 'UniqueMutuallyExclusives'.");
    rclcpp::shutdown();

    return;
  }

  // 100 Hz time
  timer_1_ = create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&TimerExample::callback_timer1, this), cb_grp1_);

  // 10 Hz timer
  timer_2_ = create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&TimerExample::callback_timer2, this), cb_grp2_);

  // 1 Hz timer with 800ms work to do, it will block the other timers from being executed because if it is in the same MutuallyExclusive callback group.
  // It will not block if it is in a Reentrant callback group or a different callback group than the other callbacks.
  timer_3_ = create_wall_timer(std::chrono::duration<double>(1.0 / 0.2), std::bind(&TimerExample::callback_timer3, this), cb_grp3_);

  // a timer that takes longer to execute than the desired rate of execution
  timer_4_ = create_wall_timer(std::chrono::duration<double>(1.0 / 0.5), std::bind(&TimerExample::callback_timer4, this), cb_grp4_);

  publisher_timer_1_ = create_publisher<std_msgs::msg::Float64>("timer_1", 10);
  publisher_timer_2_ = create_publisher<std_msgs::msg::Float64>("timer_2", 10);
  publisher_timer_3_ = create_publisher<std_msgs::msg::Float64>("timer_3", 10);
  publisher_timer_4_ = create_publisher<std_msgs::msg::Float64>("timer_3", 10);

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "[TimerExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTimer1() //{ */

void TimerExample::callback_timer1() {

  static RateCounter cntr(get_clock(), 300);
  const double       rate = cntr.update_rate();

  publisher_timer_1_->publish(std_msgs::msg::Float64().set__data(rate));

  RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "[TimerExample]: 100Hz timer spinning at rate " << rate << "Hz");
}

//}

/* callbackTimer2() //{ */

void TimerExample::callback_timer2() {

  static RateCounter cntr(get_clock(), 30);
  const double       rate = cntr.update_rate();

  publisher_timer_2_->publish(std_msgs::msg::Float64().set__data(rate));

  RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "[TimerExample]: 10Hz timer spinning at rate " << rate << "Hz");
}

//}

/* callbackTimer3() //{ */

void TimerExample::callback_timer3() {

  static RateCounter cntr(get_clock(), 5);
  const double       rate = cntr.update_rate();

  publisher_timer_3_->publish(std_msgs::msg::Float64().set__data(rate));
  RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000, "[TimerExample]: 0.2Hz timer (5s period) spinning at rate " << rate << "Hz, starting 4s work");
  rclcpp::sleep_for(std::chrono::milliseconds(4000));

  RCLCPP_INFO(get_logger(), "[TimerExample]: 0.2 Hz timer stopping work");
}

//}

/* callbackTimer4() //{ */

void TimerExample::callback_timer4() {

  static std::mutex mtx;
  {
    std::scoped_lock   lck(mtx);
    static RateCounter cntr(get_clock(), 5);
    const double       rate = cntr.update_rate();

    publisher_timer_4_->publish(std_msgs::msg::Float64().set__data(rate));
    RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
                                "[TimerExample]: 0.5Hz timer (2s period) spinning at rate " << rate << "Hz, starting 4s work");
  }

  // this sleep simulates some kind of work done by this thread that can be paralellized
  rclcpp::sleep_for(std::chrono::milliseconds(4000));

  RCLCPP_INFO(get_logger(), "[TimerExample]: 0.5 Hz timer stopping work");
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::TimerExample)
