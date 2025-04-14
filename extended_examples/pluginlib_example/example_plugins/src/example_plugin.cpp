#include <rclcpp/rclcpp.hpp>

#include <ros2_examples/params.h>
#include <example_plugin_manager/plugin_interface.h>
#include <example_plugin_manager/common_handlers.h>

namespace example_plugins
{

/* class ExamplePlugin //{ */

class ExamplePlugin : public example_plugin_manager::Plugin {

public:
  void initialize(const std::shared_ptr<rclcpp::Node> &sub_node, const std::string& name,
                  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers);

  bool activate(const int& some_number);
  void deactivate(void);

  const std::optional<double> update(const Eigen::Vector3d& input);

  // parameter from a config file
  // double _pi_;

  std::shared_ptr<rclcpp::Node> node_;
  std::string _name_;

private:
  bool is_initialized_ = false;
  bool is_active_      = false;

  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers_;
};

//}

// | -------------------- plugin interface -------------------- |

/* initialize() //{ */

void ExamplePlugin::initialize(const std::shared_ptr<rclcpp::Node> &sub_node, const std::string& name,
                               std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers) {

  node_ = sub_node;
  _name_ = name;

  // I can use this to get stuff from the manager interactively
  common_handlers_ = common_handlers;

  // | ------------------- loading parameters ------------------- |

  // bool loaded_successfully = true;

  // loaded_successfully &= utils::parse_param("pi", _pi_, *node_);

  // if (!loaded_successfully) {
  //   RCLCPP_ERROR_STREAM(node_->get_logger(), "Could not load all non-optional parameters. Shutting down.");
  //   rclcpp::shutdown();
  //   return;
  // }

  // RCLCPP_INFO(node_->get_logger(), "[%s]: loaded custom parameter: pi=%f", _name_.c_str(), _pi_);

  // | ----------------------- finish init ---------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: initialized under the name '%s', and namespace '%s'", _name_.c_str(), name.c_str(), node_->get_namespace());

  is_initialized_ = true;
}

//}

/* activate() //{ */

bool ExamplePlugin::activate(const int& some_number) {

  RCLCPP_INFO(node_->get_logger(), "[%s]: activated with some_number=%d", _name_.c_str(), some_number);

  is_active_ = true;

  return true;
}

//}

/* deactivate() //{ */

void ExamplePlugin::deactivate(void) {

  is_active_ = false;

  RCLCPP_INFO(node_->get_logger(), "[%s]: deactivated", _name_.c_str());
}

//}

/* update() //{ */

const std::optional<double> ExamplePlugin::update(const Eigen::Vector3d& input) {

  if (!is_active_) {
    return false;
  }

  RCLCPP_ERROR_STREAM(node_->get_logger(), "[" << _name_ << "]: update() was called, let's find out the size of the vector [" << input.transpose() << "]");

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

}  // namespace example_plugins

#include <pluginlib/class_list_macros.hpp>

// Register the plugin.
PLUGINLIB_EXPORT_CLASS(example_plugins::ExamplePlugin, example_plugin_manager::Plugin)