#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <example_plugin_manager/plugin_interface.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/mutex.h>

#include <pluginlib/class_loader.h>

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

class ExamplePluginManager : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;
  bool            is_initialized_ = false;

  // | ---------------------- update timer ---------------------- |

  ros::Timer timer_update_;
  double     _rate_timer_update_;

  // | -------- an object we want to share to our plugins ------- |

  std::string example_of_a_shared_object_;

  // | --------------------- common handlers -------------------- |

  std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers_;

  // | --------------- dynamic loading of plugins --------------- |

  std::unique_ptr<pluginlib::ClassLoader<example_plugin_manager::Plugin>> plugin_loader_;  // pluginlib loader
  std::vector<std::string>                                                _plugin_names_;
  std::map<std::string, PluginParams>                                     plugins_;      // map between plugin names and plugin params
  std::vector<boost::shared_ptr<example_plugin_manager::Plugin>>          plugin_list_;  // list of plugins, routines are callable from this
  std::mutex                                                              mutex_plugins_;

  std::string _initial_plugin_name_;
  int         _initial_plugin_idx_ = 0;

  int active_plugin_idx_ = 0;

  // | ------------------------ routines ------------------------ |

  double vectorNorm(const Eigen::Vector3d& input);

  // | ------------------------- timers ------------------------- |

  void timerUpdate(const ros::TimerEvent& event);
};

//}

/* onInit() //{ */

void ExamplePluginManager::onInit() {

  ros::NodeHandle nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[ExamplePluginManager]: initializing");

  // --------------------------------------------------------------
  // |                           params                           |
  // --------------------------------------------------------------

  mrs_lib::ParamLoader param_loader(nh_, "ExamplePluginManager");

  param_loader.loadParam("update_timer_rate", _rate_timer_update_);
  param_loader.loadParam("initial_plugin", _initial_plugin_name_);

  // | --------------- example of a shared object --------------- |

  example_of_a_shared_object_ = "Hello, this is a shared object";

  // --------------------------------------------------------------
  // |                       common handlers                      |
  // --------------------------------------------------------------

  common_handlers_ = std::make_shared<example_plugin_manager::CommonHandlers_t>();

  common_handlers_->some_shared_object = std::make_shared<std::string>(example_of_a_shared_object_);

  common_handlers_->vector_calculator.vectorNorm = boost::bind(&ExamplePluginManager::vectorNorm, this, _1);
  common_handlers_->vector_calculator.enabled    = true;

  // --------------------------------------------------------------
  // |                      load the plugins                      |
  // --------------------------------------------------------------

  param_loader.loadParam("plugins", _plugin_names_);

  plugin_loader_ = std::make_unique<pluginlib::ClassLoader<example_plugin_manager::Plugin>>("example_plugin_manager", "example_plugin_manager::Plugin");

  // for each plugin in the list
  for (int i = 0; i < int(_plugin_names_.size()); i++) {
    std::string plugin_name = _plugin_names_[i];

    // load the plugin parameters
    std::string address;
    std::string name_space;
    double      some_property;

    param_loader.loadParam(plugin_name + "/address", address);
    param_loader.loadParam(plugin_name + "/name_space", name_space);
    param_loader.loadParam(plugin_name + "/some_property", some_property);

    PluginParams new_plugin(address, name_space, some_property);
    plugins_.insert(std::pair<std::string, PluginParams>(plugin_name, new_plugin));

    try {
      ROS_INFO("[ExamplePluginManager]: loading the plugin '%s'", new_plugin.address.c_str());
      plugin_list_.push_back(plugin_loader_->createInstance(new_plugin.address.c_str()));
    }
    catch (pluginlib::CreateClassException& ex1) {
      ROS_ERROR("[ExamplePluginManager]: CreateClassException for the plugin '%s'", new_plugin.address.c_str());
      ROS_ERROR("[ExamplePluginManager]: Error: %s", ex1.what());
      ros::shutdown();
    }
    catch (pluginlib::PluginlibException& ex) {
      ROS_ERROR("[ExamplePluginManager]: PluginlibException for the plugin '%s'", new_plugin.address.c_str());
      ROS_ERROR("[ExamplePluginManager]: Error: %s", ex.what());
      ros::shutdown();
    }
  }

  ROS_INFO("[ExamplePluginManager]: plugins were loaded");

  for (int i = 0; i < int(plugin_list_.size()); i++) {
    try {
      std::map<std::string, PluginParams>::iterator it;
      it = plugins_.find(_plugin_names_[i]);

      ROS_INFO("[ExamplePluginManager]: initializing the plugin '%s'", it->second.address.c_str());
      plugin_list_[i]->initialize(nh_, _plugin_names_[i], it->second.name_space, common_handlers_);
    }
    catch (std::runtime_error& ex) {
      ROS_ERROR("[ExamplePluginManager]: exception caught during plugin initialization: '%s'", ex.what());
    }
  }

  ROS_INFO("[ExamplePluginManager]: plugins were initialized");

  // --------------------------------------------------------------
  // |          check for existance of the initial plugin         |
  // --------------------------------------------------------------

  {
    bool check = false;

    for (int i = 0; i < int(_plugin_names_.size()); i++) {

      std::string plugin_name = _plugin_names_[i];

      if (plugin_name == _initial_plugin_name_) {
        check                = true;
        _initial_plugin_idx_ = i;
        break;
      }
    }
    if (!check) {
      ROS_ERROR("[ExamplePluginManager]: the initial plugin (%s) is not within the loaded plugins", _initial_plugin_name_.c_str());
      ros::shutdown();
    }
  }

  // | ---------- activate the first plugin on the list --------- |

  ROS_INFO("[ExamplePluginManager]: activating plugin with idx %d on the list (named: %s)", _initial_plugin_idx_, _plugin_names_[_initial_plugin_idx_].c_str());

  int some_activation_input_to_plugin = 1234;

  plugin_list_[_initial_plugin_idx_]->activate(some_activation_input_to_plugin);
  active_plugin_idx_ = _initial_plugin_idx_;

  // | ------------------------- timers ------------------------- |

  timer_update_ = nh_.createTimer(ros::Rate(_rate_timer_update_), &ExamplePluginManager::timerUpdate, this);

  // | ----------------------- finish init ---------------------- |

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[ExamplePluginManager]: could not load all parameters!");
    ros::shutdown();
  }

  is_initialized_ = true;

  ROS_INFO("[ExamplePluginManager]: initialized");
}

//}

// | ------------------------- timers ------------------------- |

/* timerUpdate() //{ */

void ExamplePluginManager::timerUpdate([[maybe_unused]] const ros::TimerEvent& event) {

  if (!is_initialized_)
    return;

  auto active_plugin_idx = mrs_lib::get_mutexed(mutex_plugins_, active_plugin_idx_);

  // plugin input
  Eigen::Vector3d input;
  input << 0, 1, 2;

  // call the plugin's update routine
  auto result = plugin_list_[active_plugin_idx]->update(input);

  if (result) {

    // print the result
    ROS_INFO("[ExamplePluginManager]: plugin update() returned: %.2f", result.value());

  } else {

    ROS_ERROR("[ExamplePluginManager]: plugin update failed!");
  }
}

//}

// | ------------------------ routines ------------------------ |

/* vectorNorm() //{ */

double ExamplePluginManager::vectorNorm(const Eigen::Vector3d& input) {

  ROS_INFO("[ExamplePluginManager]: somebody called my vectorNorm() function, probably some plugin");

  return input.norm();
}

//}

}  // namespace example_plugin_manager

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(example_plugin_manager::ExamplePluginManager, nodelet::Nodelet)
