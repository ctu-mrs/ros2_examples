# Pluginlib Example

A working example showcasing the [pluginlib](http://wiki.ros.org/pluginlib) feature of ROS, simular to what is used between the [ControlManager](https://github.com/ctu-mrs/mrs_uav_managers#controlmanager) and the [trackers](https://github.com/ctu-mrs/mrs_uav_trackers#mrs-uav-trackers-) and [controllers](https://github.com/ctu-mrs/mrs_uav_controllers#mrs-uav-controllers-) within the [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).

## example_plugin_manager package

* defines the [interface](https://github.com/ctu-mrs/example_ros_pluginlib/blob/master/example_plugin_manager/include/example_plugin_manager/plugin_interface.h) for plugins
* defines a [common handlers](https://github.com/ctu-mrs/example_ros_pluginlib/blob/master/example_plugin_manager/include/example_plugin_manager/common_handlers.h), which are passed to the plugins
* dynamically loads the plugins defined in [plugins.yaml](https://github.com/ctu-mrs/example_ros_pluginlib/blob/master/example_plugin_manager/config/plugins.yaml) and [example_plugin_manager.yaml](https://github.com/ctu-mrs/example_ros_pluginlib/blob/master/example_plugin_manager/config/example_plugin_manager.yaml)
* activates the plugin defined in [example_plugin_manager.yaml](https://github.com/ctu-mrs/example_ros_pluginlib/blob/master/example_plugin_manager/config/example_plugin_manager.yaml)
* regularly updates the active plugin and queries a result

## example_plugins package

* defines a plugin complying with the [interface](https://github.com/ctu-mrs/example_ros_pluginlib/blob/master/example_plugin_manager/include/example_plugin_manager/plugin_interface.h)
* the plugin loads its params and prepares itself for activation
* it calculates results in its `update()` method and returns them to the manager

# How to start it?

```bash
roslaunch example_plugin_manager example_plugin_manager.launch
```

# Dependencies

* [mrs_lib](https://github.com/ctu-mrs/mrs_lib) for param loading and mutexing
