/* includes //{ */

/* each ros package must have these */
#include <rclcpp/rclcpp.hpp>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/msg/odometry.hpp>

/* custom msgs of MRS group */
#include <mrs_msgs/msg/reference_stamped.hpp>

/* for calling simple ros services */
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

//}

namespace example_waypoint_flier_simple
{

/* class WaypointFlierSimple //{ */

/* all ROS2 nodes must extend the rclcpp::Node class */
class WaypointFlierSimple : public rclcpp::Node {

public:
  WaypointFlierSimple(rclcpp::NodeOptions options);

private:
  /* node handle */
  rclcpp::Node::SharedPtr node_;

  /* clock handle */
  rclcpp::Clock::SharedPtr clock_;

  /* all callbacks belonging to the same callback group are called in sequence */
  rclcpp::CallbackGroup::SharedPtr cbkgrp_main_;
  // | -------------------------- flags ------------------------- |

  /* is set to true when the nodelet is initialized, useful for rejecting callbacks that are called before the node is initialized */
  std::atomic<bool> is_initialized_ = false;

  /* by default, the nodelet is deactivated, it only starts publishing goals when activated */
  std::atomic<bool> active_ = false;

  /* by default, the nodelet is deactivated, it only starts publishing goals when activated */
  std::atomic<bool> have_odom_ = false;

  /* variables to store the coordinates of the current goal */
  double goal_x_ = 0.0;
  double goal_y_ = 0.0;
  double goal_z_ = 2.0;

  /* variables to store the maximum limit for the random waypoint generator */
  double max_x_;
  double max_y_;
  double max_z_;

  /* ROS messages which store the current reference and odometry */
  mrs_msgs::msg::ReferenceStamped ref_;
  nav_msgs::msg::Odometry         current_odom_;

  // | ---------------------- ROS subscribers --------------------- |
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr sub_odom_;
  void                                                     callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);

  // | ---------------------- ROS publishers --------------------- |
  rclcpp::Publisher<mrs_msgs::msg::ReferenceStamped>::SharedPtr pub_reference_;

  // | ---------------------- ROS timers --------------------- |
  rclcpp::TimerBase::SharedPtr timer_initialization_;
  void                         timerInitialization();

  rclcpp::TimerBase::SharedPtr timer_main_;
  void                         timerMain();

  // | ---------------------- ROS service servers --------------------- |
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_start_;
  bool                                               callbackStart([[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // | ------------------ Additional functions ------------------ |
  double distance(const mrs_msgs::msg::ReferenceStamped& waypoint, const nav_msgs::msg::Odometry& odom);

  double getRandomDouble(double min, double max);
};
//}

/*//{ WaypointFlierSimple() */
WaypointFlierSimple::WaypointFlierSimple(rclcpp::NodeOptions options) : Node("waypoint_flier_simple", options) {

  /* constructor should not take too much computation time, so we start a timer that will handle the node initialization */
  timer_initialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&WaypointFlierSimple::timerInitialization, this));
}
/*//}*/

/* timerInitialization() //{ */

void WaypointFlierSimple::timerInitialization() {

  /* obtain node handle */
  node_ = this->shared_from_this();

  /* obtain clock handle */
  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "[WaypointFlierSimple]: initializing");

  /* initialize random number generator */
  srand(time(NULL));


  // | ------------------- load ros parameters ------------------ |
  this->declare_parameter("max_x", 10.0);
  this->declare_parameter("max_y", 10.0);
  this->declare_parameter("max_z", 5.0);

  max_x_ = this->get_parameter("max_x").as_double();
  max_y_ = this->get_parameter("max_y").as_double();
  max_z_ = this->get_parameter("max_z").as_double();

  // | -------- initialize a subscriber for UAV odometry -------- |
  /* create mutually exclusive callback group (cannot call two callbacks at the same time) */
  cbkgrp_main_           = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto sub_opt           = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = cbkgrp_main_;

  /* bind the member callback method to a function that can be passed to the subscriber */
  const std::function<void(const nav_msgs::msg::Odometry::ConstSharedPtr)> odom_callback = std::bind(&WaypointFlierSimple::callbackOdom, this, std::placeholders::_1);

  /* create the subscriber */
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>("~/odom_in", 10, odom_callback, sub_opt);

  // | -------- initialize a publisher for UAV reference -------- |
  pub_reference_ = create_publisher<mrs_msgs::msg::ReferenceStamped>("~/reference_out", 1);

  // | -- initialize the main timer - main loop of the nodelet -- |
  timer_main_ = create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&WaypointFlierSimple::timerMain, this), cbkgrp_main_);

  // | ---------------- initialize service server --------------- |
  srv_server_start_ = create_service<std_srvs::srv::Trigger>("~/start", std::bind(&WaypointFlierSimple::callbackStart, this, std::placeholders::_1, std::placeholders::_2));

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "[WaypointFlierSimple]: initialized");

  /* the initialization is finished so we cancel the timer so that it is not run again */
  timer_initialization_->cancel();
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackOdom() //{ */


void WaypointFlierSimple::callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }

  // | -------------- save the current UAV odometry ------------- |
  current_odom_ = *msg;
  have_odom_    = true;
}

//}

// | --------------------- timer callbacks -------------------- |

/* timerMain() //{ */

void WaypointFlierSimple::timerMain() {

  if (!active_) {

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1.0, "[WaypointFlierSimple]: Waiting for activation");

  } else {

    // calculate the distance to the current reference
    double dist_to_ref = distance(ref_, current_odom_);
    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1.0, "[WaypointFlierSimple]: Distance to reference: " << dist_to_ref);

    // if the distance is less than 1 meter, generate a new reference
    if (dist_to_ref < 1.0) {

      RCLCPP_INFO_STREAM(node_->get_logger(), "[WaypointFlierSimple]: Goal reached!");

      // generate new goal
      goal_x_ = getRandomDouble(-max_x_, max_x_);
      goal_y_ = getRandomDouble(-max_y_, max_y_);
      goal_z_ = getRandomDouble(2.0, max_z_);

      RCLCPP_INFO_STREAM(node_->get_logger(), "[WaypointFlierSimple]: New goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);
    }

    // fill out the ROS message and publish it
    ref_.reference.position.x = goal_x_;
    ref_.reference.position.y = goal_y_;
    ref_.reference.position.z = goal_z_;
    ref_.reference.heading    = 0.0;
    pub_reference_->publish(ref_);
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStart() */

bool WaypointFlierSimple::callbackStart([[maybe_unused]] std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  // | ------------------- activation service ------------------- |
  // only activates the main loop when the nodelet is initialized and receiving odometry

  if (!is_initialized_) {

    res->success = false;
    res->message = "Waypoint flier not initialized!";
    RCLCPP_WARN(node_->get_logger(), "[WaypointFlierSimple]: Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (!have_odom_) {

    res->success = false;
    res->message = "Waypoint flier is not receiving odometry!";
    RCLCPP_WARN(node_->get_logger(), "[WaypointFlierSimple]: Cannot start, nodelet is not receiving odometry!");
    return true;
  }

  active_ = true;

  RCLCPP_INFO(node_->get_logger(), "[WaypointFlierSimple]: Starting waypoint following.");
  RCLCPP_INFO_STREAM(node_->get_logger(), "[WaypointFlierSimple]: Goal X: " << goal_x_ << " Y: " << goal_y_ << " Z: " << goal_z_);

  res->success = true;
  res->message = "Starting waypoint following.";

  return true;
}

//}

// | -------------------- support functions ------------------- |

/*//{ distance() */

double WaypointFlierSimple::distance(const mrs_msgs::msg::ReferenceStamped& waypoint, const nav_msgs::msg::Odometry& odom) {

  // | ------------- distance between two 3D points ------------- |

  return sqrt((pow(waypoint.reference.position.x - odom.pose.pose.position.x, 2)) + (pow(waypoint.reference.position.y - odom.pose.pose.position.y, 2)) + (pow(waypoint.reference.position.z - odom.pose.pose.position.z, 2)));
}

/*//}*/

/*//{ getRandomDouble() */
double WaypointFlierSimple::getRandomDouble(double min, double max) {


  float r = (float)rand() / (float)RAND_MAX;
  return min + r * (max - min);
}
/*//}*/

}  // namespace example_waypoint_flier_simple

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(example_waypoint_flier_simple::WaypointFlierSimple)
