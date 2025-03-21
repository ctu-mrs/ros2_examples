#include <memory>
#include <string>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_loader.hpp>

#include <ros2_examples/params.h>
#include <example_plugin_manager/plugin_interface.h>

namespace example_plugin_manager
{

/* //{ class ExamplePluginManager */

/* class PluginParams() //{ */

class PluginParams {

public:
  PluginParams(const std::string& address, const std::string& name_space, const double& some_property);

public:
  std::string address;
  std::string name_space;
  double      some_property;
};

PluginParams::PluginParams(const std::string& address, const std::string& name_space, const double& some_property) {

  this->address       = address;
  this->name_space    = name_space;
  this->some_property = some_property;
}

//}

class ExamplePluginManager : public rclcpp::Node {

public:
  ExamplePluginManager(const rclcpp::NodeOptions& options);

private:
  bool        is_initialized_ = false;
  std::string _uav_type_      = "";

  // | ---------------------- update timer ---------------------- |

  rclcpp::TimerBase::SharedPtr timer_update_;
  double                       _rate_timer_update_;

  // | -------- an object we want to share to our plugins ------- |

  std::string example_of_a_shared_object_;

  // | --------------------- common handlers -------------------- |

  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers_;

  // | --------------- dynamic loading of plugins --------------- |

  std::unique_ptr<pluginlib::ClassLoader<example_plugin_manager::Plugin>> plugin_loader_;  // pluginlib loader
  std::vector<std::string>                                                _plugin_names_;
  std::map<std::string, PluginParams>                                     plugins_;      // map between plugin names and plugin params
  std::vector<std::shared_ptr<example_plugin_manager::Plugin>>            plugin_list_;  // list of plugins, routines are callable from this
  // std::mutex                                                              mutex_plugins_;

  std::string _initial_plugin_name_;
  int         _initial_plugin_idx_ = 0;

  int active_plugin_idx_ = 0;

  // | ------------------------ routines ------------------------ |

  double vectorNorm(const Eigen::Vector3d& input);

  // | ------------------------- timers ------------------------- |

  void timerUpdate();
};

//}

/* onInit() //{ */

ExamplePluginManager::ExamplePluginManager(const rclcpp::NodeOptions& options) : Node("example_plugin_manager", options) {

  RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: initializing");

  // | --------------------- load parameters -------------------- |

  bool loaded_successfully = true;

  loaded_successfully &= utils::load_param("uav_type", _uav_type_, *this);
  loaded_successfully &= utils::load_param("update_timer_rate", _rate_timer_update_, *this);
  loaded_successfully &= utils::load_param("initial_plugin", _initial_plugin_name_, *this);

  if (!loaded_successfully) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
    return;
  }
  // | --------------- example of a shared object --------------- |

  example_of_a_shared_object_ = "Hello, this is a shared object";

  // --------------------------------------------------------------
  // |                       common handlers                      |
  // --------------------------------------------------------------

  common_handlers_ = std::make_shared<example_plugin_manager::CommonHandlers_t>();

  common_handlers_->some_shared_object = std::make_shared<std::string>(example_of_a_shared_object_);

  common_handlers_->vector_calculator.vectorNorm = std::bind(&ExamplePluginManager::vectorNorm, this, std::placeholders::_1);
  common_handlers_->vector_calculator.enabled    = true;

  // --------------------------------------------------------------
  // |                      load the plugins                      |
  // --------------------------------------------------------------

  loaded_successfully &= utils::load_param("plugins", _plugin_names_, *this);

  if (!loaded_successfully) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<example_plugin_manager::Plugin>>("example_plugin_manager", "example_plugin_manager::Plugin");

  // for each plugin in the list
  for (int i = 0; i < int(_plugin_names_.size()); i++) {
    std::string plugin_name = _plugin_names_.at(i);

    // load the plugin parameters
    std::string address;
    std::string name_space;
    double      some_property;

    // nested params are separated by '.' instead of '/'
    loaded_successfully &= utils::load_param(plugin_name + ".address", address, *this);
    loaded_successfully &= utils::load_param(plugin_name + ".name_space", name_space, *this);
    loaded_successfully &= utils::load_param(plugin_name + ".some_property", some_property, *this);

    if (!loaded_successfully) {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    PluginParams new_plugin(address, name_space, some_property);
    plugins_.insert(std::pair<std::string, PluginParams>(plugin_name, new_plugin));

    try {
      RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: loading the plugin '%s'", new_plugin.address.c_str());
      plugin_list_.emplace_back(plugin_loader_->createSharedInstance(new_plugin.address));
    }
    catch (pluginlib::CreateClassException& ex1) {
      RCLCPP_ERROR(get_logger(), "[ExamplePluginManager]: CreateClassException for the plugin '%s'", new_plugin.address.c_str());
      RCLCPP_ERROR(get_logger(), "[ExamplePluginManager]: Error: %s", ex1.what());
      rclcpp::shutdown();
      return;
    }
    catch (pluginlib::PluginlibException& ex) {
      RCLCPP_ERROR(get_logger(), "[ExamplePluginManager]: PluginlibException for the plugin '%s'", new_plugin.address.c_str());
      RCLCPP_ERROR(get_logger(), "[ExamplePluginManager]: Error: %s", ex.what());
      rclcpp::shutdown();
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: plugins were loaded");

  for (int i = 0; i < int(plugin_list_.size()); i++) {
    try {
      std::map<std::string, PluginParams>::iterator it;
      it = plugins_.find(_plugin_names_.at(i));

      RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: initializing the plugin '%s' of type '%s'", it->second.name_space.c_str(), it->second.address.c_str());
      plugin_list_.at(i)->initialize(this->create_sub_node(it->second.name_space), _plugin_names_.at(i), common_handlers_);
    }
    catch (std::runtime_error& ex) {
      RCLCPP_ERROR(get_logger(), "[ExamplePluginManager]: exception caught during plugin initialization: '%s'", ex.what());
    }
  }

  RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: plugins were initialized");

  // --------------------------------------------------------------
  // |          check for existance of the initial plugin         |
  // --------------------------------------------------------------

  {
    bool check = false;

    for (int i = 0; i < int(_plugin_names_.size()); i++) {

      std::string plugin_name = _plugin_names_.at(i);

      if (plugin_name == _initial_plugin_name_) {
        check                = true;
        _initial_plugin_idx_ = i;
        break;
      }
    }
    if (!check) {
      RCLCPP_ERROR(get_logger(), "[ExamplePluginManager]: the initial plugin (%s) is not within the loaded plugins", _initial_plugin_name_.c_str());
      rclcpp::shutdown();
      return;
    }
  }

  // | ---------- activate the first plugin on the list --------- |

  RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: activating plugin with idx %d on the list (named: %s)", _initial_plugin_idx_,
              _plugin_names_.at(_initial_plugin_idx_).c_str());

  int some_activation_input_to_plugin = 1234;

  plugin_list_.at(_initial_plugin_idx_)->activate(some_activation_input_to_plugin);
  active_plugin_idx_ = _initial_plugin_idx_;

  // | ------------------------- timers ------------------------- |

  timer_update_ = create_wall_timer(std::chrono::duration<double>(1.0 / _rate_timer_update_), std::bind(&ExamplePluginManager::timerUpdate, this));

  // | ----------------------- finish init ---------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerUpdate() //{ */

void ExamplePluginManager::timerUpdate() {

  if (!is_initialized_)
    return;

  // plugin input
  Eigen::Vector3d input;
  input << 0, 1, 2;

  // call the plugin's update routine
  auto result = plugin_list_.at(active_plugin_idx_)->update(input);

  if (result) {

    // print the result
    RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: plugin update() returned: %.2f", result.value());

  } else {

    RCLCPP_ERROR(get_logger(), "[ExamplePluginManager]: plugin update failed!");
  }
}

//}

// | ------------------------ routines ------------------------ |

/* vectorNorm() //{ */

double ExamplePluginManager::vectorNorm(const Eigen::Vector3d& input) {

  RCLCPP_INFO(get_logger(), "[ExamplePluginManager]: somebody called my vectorNorm() function, probably some plugin");

  return input.norm();
}

//}

}  // namespace example_plugin_manager
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(example_plugin_manager::ExamplePluginManager)
