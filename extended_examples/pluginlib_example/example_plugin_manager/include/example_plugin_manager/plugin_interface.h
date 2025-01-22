#ifndef PLUGIN_INTERFACE_H
#define PLUGIN_INTERFACE_H

#include <ros/ros.h>

#include <example_plugin_manager/common_handlers.h>

namespace example_plugin_manager
{

class Plugin {
public:
  virtual void initialize(const ros::NodeHandle& parent_nh, const std::string& name, const std::string& name_space,
                          std::shared_ptr<example_plugin_manager::CommonHandlers_t> common_handlers) = 0;

  virtual bool activate(const int& some_number) = 0;

  virtual void deactivate(void) = 0;

  virtual const std::optional<double> update(const Eigen::Vector3d& input) = 0;

  virtual ~Plugin() = default;
};

}  // namespace example_plugin_manager

#endif
