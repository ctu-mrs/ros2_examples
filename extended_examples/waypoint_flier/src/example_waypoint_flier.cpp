/* includes //{ */
/* each ros package must have these */
#include <rclcpp/rclcpp.hpp>

/* for loading dynamic parameters while the nodelet is running */
/* #include <dynamic_reconfigure/server.h> */

/* this header file is created during compilation from python script dynparam.cfg */
/* #include <example_waypoint_flier/dynparamConfig.h> */

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/msg/odometry.hpp>

/* custom msgs of MRS group */
#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/reference_stamped.hpp>
#include <mrs_msgs/srv/string.hpp>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/service_server_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
/* #include <mrs_lib/msg_extractor.h> */
#include <mrs_lib/geometry/misc.h>

/* for calling simple ros services */
#include <std_srvs/srv/trigger.hpp>

/* for operations with matrices */
#include <Eigen/Dense>

//}

/*//{ using */
using vec2_t = mrs_lib::geometry::vec_t<2>;
using vec3_t = mrs_lib::geometry::vec_t<3>;
/*//}*/

/* typedefs //{ */
#if USE_ROS_TIMER == 1
typedef mrs_lib::ROSTimer TimerType;
#else
typedef mrs_lib::ThreadTimer TimerType;
#endif
//}

namespace example_waypoint_flier
{

/* class ExampleWaypointFlier //{ */
class ExampleWaypointFlier : public rclcpp::Node {

public:
  ExampleWaypointFlier(rclcpp::NodeOptions options);

private:
  /* node handle */
  rclcpp::Node::SharedPtr node_;

  /* clock handle */
  rclcpp::Clock::SharedPtr clock_;

  /* all callbacks belonging to the same callback group are called in sequence */
  rclcpp::CallbackGroup::SharedPtr cbkgrp_main_;

  /* flags */
  std::atomic<bool> is_initialized_ = false;

  /* ros parameters */
  std::string _uav_name_;
  std::string _config_file_;

  // | ------------------ initialization timer ------------------ |
  rclcpp::TimerBase::SharedPtr timer_initialization_;
  void                         timerInitialization();

  // | ---------------------- msg callbacks --------------------- |

  mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>                  sh_odometry_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;

  void              callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr msg);
  std::atomic<bool> have_goal_        = false;
  std::atomic<bool> waypoint_reached_ = false;

  // | --------------------- timer callbacks -------------------- |

  void                                                     timerPublishDistToWaypoint();
  mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped> ph_dist_to_waypoint_;
  std::shared_ptr<TimerType>                               timer_publish_dist_to_waypoint_;
  int                                                      _rate_timer_publish_dist_to_waypoint_;

  void                                                       timerPublishSetReference();
  mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped> ph_reference_;
  std::shared_ptr<TimerType>                                 timer_publisher_reference_;
  int                                                        _rate_timer_publisher_reference_;

  void                       timerCheckSubscribers();
  std::shared_ptr<TimerType> timer_check_subscribers_;
  int                        _rate_timer_check_subscribers_;

  // | ---------------- service server callbacks ---------------- |

  bool                                               callbackStartWaypointFollowing(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_start_waypoints_following_;

  bool                                               callbackStopWaypointFollowing(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_stop_waypoints_following_;

  bool                                               callbackFlyToFirstWaypoint(const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_server_fly_to_first_waypoint_;

  // | --------------------- service clients -------------------- |

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> srv_client_land_;
  bool                                                  _land_end_;

  // | -------------------- loading waypoints ------------------- |

  std::vector<mrs_msgs::msg::Reference> waypoints_;
  std::string                           _waypoints_frame_;
  bool                                  waypoints_loaded_ = false;
  mrs_msgs::msg::Reference              current_waypoint_;
  std::mutex                            mutex_current_waypoint_;
  int                                   idx_current_waypoint_;
  int                                   n_waypoints_;
  int                                   _n_loops_;
  int                                   c_loop_;
  std::mutex                            mutex_waypoint_idle_time_;
  Eigen::MatrixXd                       _offset_;

  // | ------------------- dynamic reconfigure ------------------ |

  /* TODO dynamic reconfigure */
  /* typedef example_waypoint_flier::dynparamConfig                              Config; */
  /* typedef dynamic_reconfigure::Server<example_waypoint_flier::dynparamConfig> ReconfigureServer; */
  /* boost::recursive_mutex                                                      mutex_dynamic_reconfigure_; */
  /* boost::shared_ptr<ReconfigureServer>                                        reconfigure_server_; */
  /* void                                                                        callbackDynamicReconfigure(Config& config, uint32_t level); */
  /* example_waypoint_flier::dynparamConfig                                      last_drs_config_; */

  // | --------------------- waypoint idling -------------------- |

  bool                         is_idling_ = false;
  rclcpp::TimerBase::SharedPtr timer_idling_;
  double                       _waypoint_idle_time_;
  double                       _waypoint_desired_dist_;
  void                         timerIdling();

  // | -------------------- support functions ------------------- |

  std::vector<mrs_msgs::msg::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);

  void offsetPoints(std::vector<mrs_msgs::msg::Reference>& points, const Eigen::MatrixXd& offset);

  double distance(const mrs_msgs::msg::Reference& waypoint, const geometry_msgs::msg::Pose& pose);
};

//}

/*//{ ExampleWaypointFlier() */
ExampleWaypointFlier::ExampleWaypointFlier(rclcpp::NodeOptions options) : Node("example_waypoint_flier", options) {

  /* constructor should not take too much computation time, so we start a timer that will handle the node initialization */
  timer_initialization_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&ExampleWaypointFlier::timerInitialization, this));
}
/*//}*/

/* timerInitialization() //{ */
void ExampleWaypointFlier::timerInitialization() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here

  have_goal_        = false;
  is_idling_        = false;
  waypoints_loaded_ = false;

  /* obtain node handle */
  node_ = this->shared_from_this();

  /* obtain clock handle */
  clock_ = node_->get_clock();

  RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: initializing");

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(node_, "ExampleWaypointFlier");

  /* add config file with parameters */
  param_loader.loadParam("config", _config_file_);
  if (_config_file_ != "") {
    param_loader.addYamlFile(_config_file_);
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("land_at_the_end", _land_end_);
  param_loader.loadParam("n_loops", _n_loops_);
  param_loader.loadParam("waypoint_desired_distance", _waypoint_desired_dist_);
  param_loader.loadParam("waypoint_idle_time", _waypoint_idle_time_);
  param_loader.loadParam("waypoints_frame", _waypoints_frame_);
  param_loader.loadParam("rate/publish_dist_to_waypoint", _rate_timer_publish_dist_to_waypoint_);
  param_loader.loadParam("rate/publish_reference", _rate_timer_publisher_reference_);
  param_loader.loadParam("rate/check_subscribers", _rate_timer_check_subscribers_);

  /* load waypoints as a half-dynamic matrix from config file */
  Eigen::MatrixXd waypoint_matrix;
  param_loader.loadMatrixDynamic("waypoints", waypoint_matrix, -1, 4);  // -1 indicates the dynamic dimension
  waypoints_            = matrixToPoints(waypoint_matrix);
  n_waypoints_          = waypoints_.size();
  waypoints_loaded_     = true;
  idx_current_waypoint_ = 0;
  c_loop_               = 0;
  RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "[ExampleWaypointFlier]: " << n_waypoints_ << " waypoints loaded");
  RCLCPP_INFO_STREAM_ONCE(node_->get_logger(), "[ExampleWaypointFlier]: " << _n_loops_ << " loops requested");

  /* load offset of all waypoints as a static matrix from config file */
  param_loader.loadMatrixKnown("offset", _offset_, 1, 4);
  offsetPoints(waypoints_, _offset_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "[ExampleWaypointFlier]: failed to load non-optional parameters!");
    rclcpp::shutdown();
  }

  // | ------------------ initialize subscribers ----------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node                                = node_;
  shopts.node_name                           = "ExampleWaypointFlier";
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_main_;

  sh_odometry_             = mrs_lib::SubscriberHandler<nav_msgs::msg::Odometry>(shopts, "~/odom_uav_in");
  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in", &ExampleWaypointFlier::callbackControlManagerDiag, this);

  // | ------------------ initialize publishers ----------------- |

  ph_dist_to_waypoint_ = mrs_lib::PublisherHandler<mrs_msgs::msg::Float64Stamped>(node_, "~/dist_to_waypoint_out");
  ph_reference_        = mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped>(node_, "~/reference_out");

  // | -------------------- initialize timers ------------------- |

  {
    mrs_lib::TimerHandlerOptions opts;

    opts.node                          = node_;
    opts.autostart                     = true;
    std::function<void()> callback_fcn = std::bind(&ExampleWaypointFlier::timerPublishDistToWaypoint, this);

    timer_publish_dist_to_waypoint_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_rate_timer_publish_dist_to_waypoint_, clock_), callback_fcn);
  }

  {
    mrs_lib::TimerHandlerOptions opts;

    opts.node                          = node_;
    opts.autostart                     = true;
    std::function<void()> callback_fcn = std::bind(&ExampleWaypointFlier::timerCheckSubscribers, this);

    timer_check_subscribers_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_rate_timer_check_subscribers_, clock_), callback_fcn);
  }


  // disable autostart of the timer
  {
    mrs_lib::TimerHandlerOptions opts;

    opts.node      = node_;
    opts.autostart = false;

    std::function<void()> callback_fcn = std::bind(&ExampleWaypointFlier::timerPublishSetReference, this);

    timer_publisher_reference_ = std::make_shared<TimerType>(opts, rclcpp::Rate(_rate_timer_publisher_reference_, clock_), callback_fcn);
  }

  // | --------------- initialize service servers --------------- |

  srv_server_start_waypoints_following_ = node_->create_service<std_srvs::srv::Trigger>("~/start_waypoints_following_in", std::bind(&ExampleWaypointFlier::callbackStartWaypointFollowing, this, std::placeholders::_1, std::placeholders::_2));
  srv_server_stop_waypoints_following_  = node_->create_service<std_srvs::srv::Trigger>("~/stop_waypoints_following_in", std::bind(&ExampleWaypointFlier::callbackStopWaypointFollowing, this, std::placeholders::_1, std::placeholders::_2));
  srv_server_fly_to_first_waypoint_     = node_->create_service<std_srvs::srv::Trigger>("~/fly_to_first_waypoint_in", std::bind(&ExampleWaypointFlier::callbackFlyToFirstWaypoint, this, std::placeholders::_1, std::placeholders::_2));

  // | --------------- initialize service clients --------------- |

  srv_client_land_ = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "~/land_out");

  // | ---------- initialize dynamic reconfigure server --------- |

  /* reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh)); */
  /* ReconfigureServer::CallbackType f = boost::bind(&ExampleWaypointFlier::callbackDynamicReconfigure, this, _1, _2); */
  /* reconfigure_server_->setCallback(f); */

  /* set the default value of dynamic reconfigure server to the value of parameter with the same name */
  /* { */
  /*   std::scoped_lock lock(mutex_waypoint_idle_time_); */
  /*   last_drs_config_.waypoint_idle_time = _waypoint_idle_time_; */
  /* } */
  /* reconfigure_server_->updateConfig(last_drs_config_); */

  RCLCPP_INFO_ONCE(node_->get_logger(), "[ExampleWaypointFlier]: initialized");

  is_initialized_ = true;

  timer_initialization_->cancel();
}

//}

// | ---------------------- msg callbacks --------------------- |

/* callbackControlManagerDiag() //{ */
void ExampleWaypointFlier::callbackControlManagerDiag(const mrs_msgs::msg::ControlManagerDiagnostics::ConstSharedPtr diagnostics) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_) {
    return;
  }

  /* do not calculate distance to waypoint when the uav is not flying towards a waypoint */
  if (!have_goal_) {
    return;
  }

  // this routine can not work without the odometry
  if (!sh_odometry_.hasMsg()) {
    return;
  }

  // get the variable under the mutex
  mrs_msgs::msg::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::msg::Pose current_pose = sh_odometry_.getMsg()->pose.pose;

  double dist = distance(current_waypoint, current_pose);

  if (have_goal_ && !diagnostics->tracker_status.have_goal) {
    have_goal_ = false;

    if (dist < _waypoint_desired_dist_) {
      waypoint_reached_ = true;
      RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Waypoint reached.");

      /* start idling at the reached waypoint */
      is_idling_ = true;

      timer_idling_ = create_wall_timer(std::chrono::duration<double>(1.0), std::bind(&ExampleWaypointFlier::timerIdling, this));

    }
  }
}

//}

// | --------------------- timer callbacks -------------------- |

/* timerPublishSetReference() //{ */
void ExampleWaypointFlier::timerPublishSetReference() {

  if (!is_initialized_) {
    return;
  }

  /* return if the uav is still flying to the previous waypoints */
  if (have_goal_) {
    return;
  }

  /* return if the UAV is idling at a waypoint */
  if (is_idling_) {
    return;
  }

  /* shutdown node after flying through all the waypoints (call land service before) */
  if (idx_current_waypoint_ >= n_waypoints_) {

    c_loop_++;

    RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Finished loop %d/%d", c_loop_, _n_loops_);

    if (c_loop_ >= _n_loops_) {

      RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Finished %d loops of %d waypoints.", _n_loops_, n_waypoints_);

      if (_land_end_) {
        RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Calling land service.");
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto                                                   res = srv_client_land_.callSync(req);

        if (res) {
          if (res.value()->success) {
            RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Land service called successfully.");
          } else {
            RCLCPP_ERROR(node_->get_logger(), "[ExampleWaypointFlier]: Land service failed with error: %s", res.value()->message.c_str());
          }
        } else {
          RCLCPP_ERROR(node_->get_logger(), "[ExampleWaypointFlier]: Could not call land service");
        }
      }

      RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Shutting down.");
      rclcpp::shutdown();
      return;

    } else {
      RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Starting loop %d/%d", c_loop_ + 1, _n_loops_);
      idx_current_waypoint_ = 0;
    }
  }

  /* create new waypoint msg */
  mrs_msgs::msg::ReferenceStamped new_waypoint;

  // set the frame id in which the reference is expressed
  new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  new_waypoint.header.stamp    = clock_->now();

  new_waypoint.reference = waypoints_.at(idx_current_waypoint_);

  // set the variable under the mutex
  mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(idx_current_waypoint_), current_waypoint_);

  RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Flying to waypoint %d: x: %.2f y: %.2f z: %.2f heading: %.2f", idx_current_waypoint_ + 1, new_waypoint.reference.position.x, new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.heading);

  ph_reference_.publish(new_waypoint);

  if (waypoint_reached_) {
    idx_current_waypoint_++;
    waypoint_reached_ = false;
  }

  have_goal_ = true;
}
//}

/* timerPublishDistToWaypoint() //{ */
void ExampleWaypointFlier::timerPublishDistToWaypoint() {

  if (!is_initialized_) {
    return;
  }

  /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
  if (!have_goal_) {
    return;
  }

  // this routine can not work without the odometry
  if (!sh_odometry_.hasMsg()) {
    return;
  }

  // get the variable under the mutex
  mrs_msgs::msg::Reference current_waypoint = mrs_lib::get_mutexed(mutex_current_waypoint_, current_waypoint_);

  // extract the pose part of the odometry
  geometry_msgs::msg::Pose current_pose = sh_odometry_.getMsg()->pose.pose;

  double dist = distance(current_waypoint, current_pose);
  RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Distance to waypoint: %.2f", dist);

  mrs_msgs::msg::Float64Stamped dist_msg;
  // it is important to set the frame id correctly !!
  dist_msg.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
  dist_msg.header.stamp    = clock_->now();
  dist_msg.value           = dist;

  ph_dist_to_waypoint_.publish(dist_msg);
}
//}

/* timerCheckSubscribers() //{ */
void ExampleWaypointFlier::timerCheckSubscribers() {

  if (!is_initialized_) {
    return;
  }

  if (!sh_odometry_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1.0, "[ExampleWaypointFlier]: Not received uav odom msg since node launch.");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "[ExampleWaypointFlier]: Received first odometry msg");
  }

  if (!sh_control_manager_diag_.hasMsg()) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1.0, "[ExampleWaypointFlier]: Not received tracker diagnostics msg since node launch.");
  } else {
    RCLCPP_INFO_ONCE(node_->get_logger(), "[ExampleWaypointFlier]: Received first control manager diagnostics msg");
  }

  if (sh_odometry_.hasMsg() && sh_control_manager_diag_.hasMsg()) {
    RCLCPP_INFO_ONCE(node_->get_logger(), "[ExampleWaypointFlier]: Waiting for activation");
  }

}

//}

/* timerIdling() //{ */
void ExampleWaypointFlier::timerIdling() {

  RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Idling for %.2f seconds.", _waypoint_idle_time_);

  /* TODO is there a more straightforward way to sleep for a duration specified in seconds? */
  clock_->sleep_for(rclcpp::Duration(std::chrono::duration<double>(_waypoint_idle_time_)));

  RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Idling finished");
  is_idling_ = false;

  /* the idling is finished so we cancel the timer so that it is not run again */
  timer_idling_->cancel();
}
//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStartWaypointFollowing() */
bool ExampleWaypointFlier::callbackStartWaypointFollowing([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  if (!is_initialized_) {

    res->success = false;
    res->message = "Waypoint flier is not initialized!";
    RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (waypoints_loaded_) {

    timer_publisher_reference_->start();

    RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Starting waypoint following.");

    res->success = true;
    res->message = "Starting waypoint following.";

  } else {

    RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot start waypoint following, waypoints are not set.");
    res->success = false;
    res->message = "Waypoints are not set.";
  }

  return true;
}

//}

/* //{ callbackStopWaypointFollowing() */
bool ExampleWaypointFlier::callbackStopWaypointFollowing([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  if (!is_initialized_) {

    res->success = false;
    res->message = "Waypoint flier not initialized!";
    RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot stop waypoint following, nodelet is not initialized.");
    return true;
  }

  timer_publisher_reference_->stop();

  RCLCPP_INFO(node_->get_logger(), "[ExampleWaypointFlier]: Waypoint following stopped.");

  res->success = true;
  res->message = "Waypoint following stopped.";

  return true;
}

//}

/* //{ callbackFlyToFirstWaypoint() */
bool ExampleWaypointFlier::callbackFlyToFirstWaypoint([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Trigger::Request> req, const std::shared_ptr<std_srvs::srv::Trigger::Response> res) {

  if (!is_initialized_) {

    res->success = false;
    res->message = "Waypoint flier not initialized!";
    RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot start waypoint following, nodelet is not initialized.");

    return true;
  }

  if (waypoints_loaded_) {

    /* create new waypoint msg */
    mrs_msgs::msg::ReferenceStamped new_waypoint;

    // it is important to set the frame id correctly !!
    new_waypoint.header.frame_id = _uav_name_ + "/" + _waypoints_frame_;
    new_waypoint.header.stamp    = clock_->now();
    new_waypoint.reference       = waypoints_.at(0);

    // set the variable under the mutex
    mrs_lib::set_mutexed(mutex_current_waypoint_, waypoints_.at(0), current_waypoint_);

    idx_current_waypoint_ = 0;
    c_loop_               = 0;

    have_goal_ = true;

    ph_reference_.publish(new_waypoint);

    std::stringstream ss;
    ss << "Flying to first waypoint: x: " << new_waypoint.reference.position.x << ", y: " << new_waypoint.reference.position.y << ", z: " << new_waypoint.reference.position.z << ", heading: " << new_waypoint.reference.heading;

    RCLCPP_INFO_STREAM_THROTTLE(node_->get_logger(), *clock_, 1.0, "[ExampleWaypointFlier]: " << ss.str());

    res->success = true;
    res->message = ss.str();

  } else {

    RCLCPP_WARN(node_->get_logger(), "[ExampleWaypointFlier]: Cannot fly to first waypoint, waypoints are not loaded!");

    res->success = false;
    res->message = "Waypoints are not loaded";
  }

  return true;
}

//}

// | -------------- dynamic reconfigure callback -------------- |

/* //{ callbackDynamicReconfigure() */
/* void ExampleWaypointFlier::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) { */

/*   if (!is_initialized_) */
/*     return; */

/*   RCLCPP_INFO( */
/*       "[ExampleWaypointFlier]:" */
/*       "Reconfigure Request: " */
/*       "Waypoint idle time: %.2f", */
/*       config.waypoint_idle_time); */

/*   { */
/*     std::scoped_lock lock(mutex_waypoint_idle_time_); */

/*     _waypoint_idle_time_ = config.waypoint_idle_time; */
/*   } */
/* } */
//}

// | -------------------- support functions ------------------- |

/* matrixToPoints() //{ */
std::vector<mrs_msgs::msg::Reference> ExampleWaypointFlier::matrixToPoints(const Eigen::MatrixXd& matrix) {

  std::vector<mrs_msgs::msg::Reference> points;

  for (int i = 0; i < matrix.rows(); i++) {

    mrs_msgs::msg::Reference point;
    point.position.x = matrix.row(i)(0);
    point.position.y = matrix.row(i)(1);
    point.position.z = matrix.row(i)(2);
    point.heading    = matrix.row(i)(3);

    points.push_back(point);
  }

  return points;
}

//}

/* offsetPoints() //{ */
void ExampleWaypointFlier::offsetPoints(std::vector<mrs_msgs::msg::Reference>& points, const Eigen::MatrixXd& offset) {

  for (size_t i = 0; i < points.size(); i++) {

    points.at(i).position.x += offset(0);
    points.at(i).position.y += offset(1);
    points.at(i).position.z += offset(2);
    points.at(i).heading += offset(3);
  }
}

//}

/* distance() //{ */
double ExampleWaypointFlier::distance(const mrs_msgs::msg::Reference& waypoint, const geometry_msgs::msg::Pose& pose) {

  return mrs_lib::geometry::dist(vec3_t(waypoint.position.x, waypoint.position.y, waypoint.position.z), vec3_t(pose.position.x, pose.position.y, pose.position.z));
}
//}

}  // namespace example_waypoint_flier

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(example_waypoint_flier::ExampleWaypointFlier)
