#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class Tf2BroadcasterExample //{ */

class Tf2BroadcasterExample : public rclcpp::Node {
public:
  Tf2BroadcasterExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ---------------------- broadcaster ----------------------- |

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void broadcastTF(void);

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;

  void timerMain();
};

//}

/* Tf2BroadcasterExample() //{ */

Tf2BroadcasterExample::Tf2BroadcasterExample(rclcpp::NodeOptions options) : Node("tf2_broadcaster_example", options) {

  RCLCPP_INFO(this->get_logger(), "[Tf2BroadcasterExample]: initializing");

  // | ---------------------- broadcaster ----------------------- |

  tf_broadcaster_ = nullptr; // constructor requires shared_pointer to rclcpp::Node (aka 'this'), but pointer to 'this' does not yet exist

  // | -------------------------- timer ------------------------- |

  timer_main_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&Tf2BroadcasterExample::timerMain, this));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[Tf2BroadcasterExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* timerMain() //{ */

void Tf2BroadcasterExample::timerMain(void) {

  if (!is_initialized_) {
    return;
  }

  broadcastTF();
}

//}

// | ------------------------ routines ------------------------ |

/* broadcastTF() //{ */

void Tf2BroadcasterExample::broadcastTF() {

  RCLCPP_INFO(this->get_logger(), "[Tf2BroadcasterExample]: broadcasting transform");

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp    = this->get_clock()->now();
  transform_stamped.header.frame_id = "parent_frame";
  transform_stamped.child_frame_id  = "child_frame";

  transform_stamped.transform.translation.x = 10.0;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;

  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = 0.0;
  transform_stamped.transform.rotation.w = 1.0;

  if (tf_broadcaster_ == nullptr) { // initialize with shared_pointer to 'this' Node
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this->shared_from_this());
  }
  tf_broadcaster_->sendTransform(transform_stamped);
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::Tf2BroadcasterExample)
