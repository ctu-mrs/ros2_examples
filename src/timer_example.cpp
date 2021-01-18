#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace ros2_uav_example
{

/* class TimerExample //{ */

class TimerExample : public rclcpp::Node {
public:
  TimerExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::TimerBase::SharedPtr timer_2_;
  rclcpp::TimerBase::SharedPtr timer_3_;

  void callbackTimer1();
  void callbackTimer2();
  void callbackTimer3();
};

//}

/* TimerExample() //{ */

TimerExample::TimerExample(rclcpp::NodeOptions options) : Node("timer_example", options) {

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: initializing");

  // | -------------------------- timer ------------------------- |

  // 100 Hz timer
  timer_1_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&TimerExample::callbackTimer1, this));

  // 10 Hz timer
  timer_2_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&TimerExample::callbackTimer2, this));

  // 1 Hz timer with 800ms work to do, it will block the other timers from being executed :-(
  timer_3_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 1.0), std::bind(&TimerExample::callbackTimer3, this));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[TimerExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackTimer1() //{ */

void TimerExample::callbackTimer1(void) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 10 Hz Timer spinning");
}

//}

/* callbackTimer2() //{ */

void TimerExample::callbackTimer2(void) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 100 Hz Timer spinning");
}

//}

/* callbackTimer3() //{ */

void TimerExample::callbackTimer3(void) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 1 Hz timer spinning, starting work");
  rclcpp::sleep_for(std::chrono::milliseconds(800));
  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 1 Hz timer stopping work");
}

//}

}  // namespace ros2_uav_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_uav_example::TimerExample)
