#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class TimerExample //{ */

class TimerExample : public rclcpp::Node {
public:
  TimerExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ------------------------- timers ------------------------- |

  rclcpp::callback_group::CallbackGroup::SharedPtr cb_grp_;

  rclcpp::TimerBase::SharedPtr timer_1_;
  rclcpp::TimerBase::SharedPtr timer_2_;
  rclcpp::TimerBase::SharedPtr timer_3_;

  void callbackTimer1();
  void callbackTimer2();
  void callbackTimer3();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_timer_1_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_timer_2_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_timer_3_;
};

//}

/* TimerExample() //{ */

TimerExample::TimerExample(rclcpp::NodeOptions options) : Node("timer_example", options) {

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: initializing");

  // | -------------------------- timer ------------------------- |

  // create separate callback group. Default group in Node class is CallbackGroupType::MutuallyExclusive
  // (https://github.com/ros2/rclcpp/blob/9c62c1c9463cd2accee57fe157125950df50f957/rclcpp/src/rclcpp/node_interfaces/node_base.cpp#L151) explanation of
  // CallbackGroupTypes: https://roscon.ros.org/2014/wp-content/uploads/2014/07/ROSCON-2014-Why-you-want-to-use-ROS-2.pdf
  cb_grp_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::Reentrant);

  // 100 Hz time
  timer_1_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&TimerExample::callbackTimer1, this), cb_grp_);

  // 10 Hz timer
  timer_2_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&TimerExample::callbackTimer2, this), cb_grp_);

  // 1 Hz timer with 800ms work to do, it will block the other timers from being executed :-(
  timer_3_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 1.0), std::bind(&TimerExample::callbackTimer3, this), cb_grp_);


  publisher_timer_1_   = this->create_publisher<std_msgs::msg::String>("timer_1", 10);
  publisher_timer_2_   = this->create_publisher<std_msgs::msg::String>("timer_2", 10);
  publisher_timer_3_   = this->create_publisher<std_msgs::msg::String>("timer_3", 10);

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

  publisher_timer_1_->publish(std_msgs::msg::String());

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 100 Hz Timer spinning");
}

//}

/* callbackTimer2() //{ */

void TimerExample::callbackTimer2(void) {

  if (!is_initialized_) {
    return;
  }

  publisher_timer_2_->publish(std_msgs::msg::String());

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 10 Hz Timer spinning");
}

//}

/* callbackTimer3() //{ */

void TimerExample::callbackTimer3(void) {

  if (!is_initialized_) {
    return;
  }

  publisher_timer_3_->publish(std_msgs::msg::String());

  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 1 Hz timer spinning, starting work");
  rclcpp::sleep_for(std::chrono::milliseconds(800));
  RCLCPP_INFO(this->get_logger(), "[TimerExample]: 1 Hz timer stopping work");
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::TimerExample)
