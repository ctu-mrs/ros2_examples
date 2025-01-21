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

private:
  // | ---------------------- broadcaster ----------------------- |

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;

  void timer_main();
};

//}

/* Tf2BroadcasterExample() //{ */

Tf2BroadcasterExample::Tf2BroadcasterExample(rclcpp::NodeOptions options) : Node("tf2_broadcaster_example", options), tf_broadcaster_(this) {

  RCLCPP_INFO(get_logger(), "initializing");

  // | -------------------------- timer ------------------------- |

  timer_main_ = create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&Tf2BroadcasterExample::timer_main, this));

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* timer_main() //{ */

void Tf2BroadcasterExample::timer_main() {

  RCLCPP_INFO(get_logger(), "broadcasting transform");

  geometry_msgs::msg::TransformStamped transform_stamped;

  transform_stamped.header.stamp    = get_clock()->now();
  transform_stamped.header.frame_id = "parent_frame";
  transform_stamped.child_frame_id  = "child_frame";

  static double x = 0.0;
  x               = std::fmod(x + 0.5, 10);

  transform_stamped.transform.translation.x = x;
  transform_stamped.transform.translation.y = 0.0;
  transform_stamped.transform.translation.z = 0.0;

  transform_stamped.transform.rotation.x = 0.0;
  transform_stamped.transform.rotation.y = 0.0;
  transform_stamped.transform.rotation.z = 0.0;
  transform_stamped.transform.rotation.w = 1.0;

  tf_broadcaster_.sendTransform(transform_stamped);
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::Tf2BroadcasterExample)
