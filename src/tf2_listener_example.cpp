#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class Tf2ListenerExample //{ */

class Tf2ListenerExample : public rclcpp::Node {
public:
  Tf2ListenerExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ----------------------- listener ------------------------- |

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;

  void timerMain();
};

//}

/* Tf2ListenerExample() //{ */

Tf2ListenerExample::Tf2ListenerExample(rclcpp::NodeOptions options) : Node("tf2_listener_example", options) {

  RCLCPP_INFO(this->get_logger(), "[Tf2ListenerExample]: initializing");

  // | ---------------------- broadcaster ----------------------- |

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  // | -------------------------- timer ------------------------- |

  timer_main_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 100.0), std::bind(&Tf2ListenerExample::timerMain, this));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[Tf2ListenerExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* timerMain() //{ */

void Tf2ListenerExample::timerMain(void) {

  if (!is_initialized_) {
    return;
  }
  try {
    auto transform_stamped = tf_buffer_->lookupTransform("child_frame", "parent_frame", rclcpp::Time(0));
    RCLCPP_INFO(this->get_logger(),
                "[Tf2ListenerExample]: 'child_frame' -> 'parent_frame':\n\t translation: [%.2f, %.2f, %.2f]\n\t rotation: [%.2f, %.2f, %.2f, %.2f]",
                transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z,
                transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z);
  }
  catch (tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "[Tf2ListenerExample]: '%s'", ex.what());
  }
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::Tf2ListenerExample)
