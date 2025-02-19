#include <ctime>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class Tf2ListenerExample //{ */

class Tf2ListenerExample : public rclcpp::Node {
public:
  Tf2ListenerExample(rclcpp::NodeOptions options);

private:
  // | ----------------------- listener ------------------------- |

  tf2_ros::Buffer            tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;

  void timer_main();
};

//}

/* Tf2ListenerExample() //{ */

Tf2ListenerExample::Tf2ListenerExample(rclcpp::NodeOptions options)
    : Node("tf2_listener_example", options), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, false) {

  RCLCPP_INFO(get_logger(), "initializing");

  // to suppress the error message which doesn't apply to this case
  tf_buffer_.setUsingDedicatedThread(true);

      // | -------------------------- timer ------------------------- |

      timer_main_ = create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&Tf2ListenerExample::timer_main, this));

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* timer_main() //{ */

void Tf2ListenerExample::timer_main() {

  try {
    // rclcpp::Time(0) means look for the newest transformation between the two frames

    const geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer_.lookupTransform("child_frame", "parent_frame", rclcpp::Time(0));

    RCLCPP_INFO(get_logger(), "'child_frame' -> 'parent_frame':\n\t translation: [%.2f, %.2f, %.2f]\n\t rotation: [%.2f, %.2f, %.2f, %.2f]",
                transform_stamped.transform.translation.x, transform_stamped.transform.translation.y, transform_stamped.transform.translation.z,
                transform_stamped.transform.rotation.w, transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z);
  }
  catch (tf2::TransformException& ex) {
    RCLCPP_WARN_STREAM(get_logger(), "" << ex.what());
  }
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::Tf2ListenerExample)
