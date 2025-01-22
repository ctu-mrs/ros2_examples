#include <ros/ros.h>
#include <ros/package.h>

#include <example_plugin_manager/plugin_interface.h>

#include <mrs_lib/param_loader.h>

namespace example_plugins
{

namespace example_plugin
{

/* class ExamplePlugin //{ */

class ExamplePlugin : public example_plugin_manager::Plugin {

public:
  void initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& name_space,
                  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers);

  bool activate(const int& some_number);
  void deactivate(void);

  const std::optional<double> update(const Eigen::Vector3d& input);

  // parameter from a config file
  double _pi_;

  std::string _name_;

private:
  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers_;
};

//}

// | -------------------- plugin interface -------------------- |

/* initialize() //{ */

void ExamplePlugin::initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& name_space,
                               std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers) {

  // nh_ will behave just like normal NodeHandle
  ros::NodeHandle nh_(parent_nh, name_space);

  _name_ = name;

  // I can use this to get stuff from the manager interactively
  common_handlers_ = common_handlers;

  ros::Time::waitForValid();

  // | ------------------- loading parameters ------------------- |

  mrs_lib::ParamLoader param_loader(nh_, "ExamplePlugin");

  param_loader.addYamlFile(ros::package::getPath("example_plugins") + "/config/example_plugin.yaml");

  // can load params like in a ROS node
  param_loader.loadParam("pi", _pi_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[%s]: could not load all parameters!", _name_.c_str());
    ros::shutdown();
  }

  ROS_INFO("[%s]: loaded custom parameter: pi=%f", _name_.c_str(), _pi_);

  // | ----------------------- finish init ---------------------- |

  ROS_INFO("[%s]: initialized under the name '%s', and namespace '%s'", _name_.c_str(), name.c_str(), name_space.c_str());

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool ExamplePlugin::activate(const int& some_number) {

  ROS_INFO("[%s]: activated with some_number=%d", _name_.c_str(), some_number);

  is_active_ = true;

  return true;
}

//}

/* deactivate() //{ */

void ExamplePlugin::deactivate(void) {

  is_active_ = false;

  ROS_INFO("[%s]: deactivated", _name_.c_str());
}

//}

/* update() //{ */

const std::optional<double> ExamplePlugin::update(const Eigen::Vector3d& input) {

  if (!is_active_) {
    return false;
  }

  ROS_INFO_STREAM("[" << _name_ << "]: update() was called, let's find out the size of the vector [" << input.transpose() << "]");

  // check some property from the "manager"
  if (common_handlers_->vector_calculator.enabled) {

    // use a function from the common_handlers
    double vector_norm = common_handlers_->vector_calculator.vectorNorm(input);

    // we calculated our result, just return it to the manager
    return vector_norm;

  } else {

    return false;
  }
}

//}

}  // namespace example_plugin

}  // namespace example_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_plugins::example_plugin::ExamplePlugin, example_plugin_manager::Plugin)
