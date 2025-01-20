# ROS2 examples

This example package is meant to explore the possibilities of ROS2 from the point of view of current ROS1 features and how the ROS1 feature translate into the new ROS2 architecture.
We investigate the aspects that are currently utilized in [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) with the intention of a potential future transition.

**Meant to be tested on 24.04 - Jazzy**

## Minimalistic Examples

Everything is a component. We happily [nodelet everything](https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Nodelet%20Everything.html) in ROS1, so why otherwise?

* [ ] [**ServiceClientExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_client_example.cpp) - periodically calls a service
  * [ ] retreiving result synchronously needs to be done outside of ROS with waiting for the `std::future`
* [X] [**PublisherExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/publisher_example.cpp) - periodically publishes
* [X] [**ServiceServerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_server_example.cpp) - getting called, ok
* [X] [**SubscriberExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/subscriber_example.cpp) - subscribes, ok
* [X] [**TimerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/timer_example.cpp) - runs multiple timers in parallel
  * [X] ~~timer callbacks are not executed in parallel even with multi-threaded container (with multi-threaded executor, looks like the same problem can appear even with single-threaded executor)~~ --> timers were constructed as part of *mutual exclusive callbackGroup*
* [X] [**ParamsExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/params_example.cpp) - load params from yaml and launch file
  * [X] param server callback is hooked up
  * [X] custom configs work
* [X] [**Tf2BroadcasterExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/tf2_broadcaster_example.cpp) - publishing transforms, ok
  * [ ] no innovation in TF structure form ROS1, still missing the option to have multiple parents
* [X] [**Tf2ListenerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/tf2_listener_example.cpp) - receiving transforms, lookup table works, ok

### Running the examples

* [tmuxinator](https://github.com/tmuxinator/tmuxinator) [session](https://github.com/ctu-mrs/ros2_examples/blob/master/tmux/session.yml)

## First ROS2 impressions

* [ ] **colcon**, the build system... is it really the best we have got? I want my catkin back.
  * creates workspace wherever `colcon build` is called
  * therefore, cannot build in subdirectories
  * no `colcon clean`, no `colcon init`
  * overall not much user friendly
  * [ ] TODO workspace-wide profiles with custom flags
    * profiles are not supported by colcon yet. See [discussion](https://github.com/colcon/colcon-core/issues/168)
    * predefining of build setting with custom flags can be done using mixin
      * install: `sudo apt install python3-colcon-mixin`
      * REPO with readme and mixin examples: [repo](https://github.com/colcon/colcon-mixin-repository)
      * example of usage: `colcon build --mixin rel-with-deb-info`
* [X] Sourcing ROS2 workspace
  * ROS2 sourcing
```bash
  source /opt/ros/jazzy/setup.zsh
  source ~/ros2_workspace/install/setup.zsh
```
* [ ] When **ros2 launch** fails due to python error (nondescriptive), launch it as `ros2 launch -d ...`
* [ ] *Topic* the subscribers and publisher have to have compatible Quality of Service (QoS) settings -- reliable or best effort. For example RViZ is setting different profile (reliable) that is by default (best effort). [QoS](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)
* [X] **CMakeLists.txt**
  * [X] basics are fine
  * [X] building (and using) custom libraries
  * [X] building standalone executable nodelets
* [o] **Launch files**
  * [X] they are in python now
  * [X] remappings
  * [X] passing params
  * [ ] TODO passing terminal arguments not working (nonexistant??)
  * [ ] TODO connecting to existing component containers (nodelet managers)
  * [ ] TODO how to add launch prefix, e.g., `gdb`
* [X] **Subscribers** - work fine, just be aware of difference between CallbackGroupType::Reentrant and CallbackGroupType::MutuallyExclusive (default one).
* [X] **Publishers** - works fine, not a big difference from ROS1
* [o] **Service client**
  * [X] it is asynchronous only!!!!
  * [ ] make a wrapper that will default to synchronous services
* [X] **Service server** - works fine
* [X] **Timers** - works fine, just be aware of difference between CallbackGroupType::Reentrant and CallbackGroupType::MutuallyExclusive (default one).
* [O] **Parameters**
  * [X] basic params work from *yaml* and *launch*
  * [X] **Beware!** loading an empty *yaml* file causes a long and cryptic error (`[ERROR] [launch]: Caught exception in launch (see debug for traceback): 'NoneType' object has no attribute 'items'`). **Solution:** add some unused parameter to the file.
  * [X] Nesting is distinguished by "." in the code (it was "/" in ROS1)
  * [X] overloading multiple configs over each other works, the ones loaded last rule
  * [ ] TODO test more complex types: lists, matrices, ...
* [o] **DRS** - has a direct implementation in the default **parameters**
  * [X] callback hooked to the external change of the parameters works
  * [ ] validation of the parameters has to be performed in our code -> we will need a custom wrapper for that, otherwise madness [**madness**](https://github.com/alsora/ros2-code-examples/blob/master/simple_parameter/src/simple_parameter_server_node.cpp)
  * [X] ROS2 `rqt_reconfigure` works
* [X] why is the terminal output monochromatic? I want my colors back...
  * [X] `export RCUTILS_COLORIZED_OUTPUT=1`
* [ ] **ROS Time**
  * [ ] TODO test duration, rate, sleep, wall time vs. sim time (we need sim time for faster/slower than real time simulations)
    * [X]  To be able to listen sim time, parameter use_sime_time=1 has to be set individually for every node. [PR](https://github.com/ros2/rclcpp/pull/559)
    * [x]  While using sim time, the get_clock()->now() clock runs at 10 Hz by default. Need to increase the rate as described [here](https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1214#issuecomment-894212336).
* [ ] **Transformations**
  * [ ] TODO
  * [ ] I really hope that ROS2 will support more than just the **TF tree**, e.g., an **Acyclic graph**. We need more parents for a node to allow a robot to being localized within more coordinated systems at a time.
  * [ ] What about relativistic transformations? Meaning the frames could have a velocity and, therefore, we could transform a moving object (a ball) from a moving frame (a drone) to a world frame.
* [ ] **Tests**
  * [ ] TODO
* [ ] **Pluginlib**
  * [ ] TODO
* [ ] **Actionlib**
  * [ ] TODO
* [ ] **Lifecycles** (new feature)
  * [ ] TODO

## Migration Guide

* Follow [Migration guide from ROS1](https://docs.ros.org/en/foxy/Contributing/Migration-Guide.html#update-source-code) that provides most of the basic required modifications.
* More details regarding genereated C++ interfaces [here](https://design.ros2.org/articles/generated_interfaces_cpp.html)

### Messages/Services

* In order to use the messages generated in the same package we need to use the following CMake code:
  ```cmake
  rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  ```
  This finds the relevant generated C++ code from ``AddressBook.msg`` and allows your target to link against it.

  You may have noticed that this step was not necessa y when the interfaces being used were from a package that was built separately. This CMake code is only     required when you want to use interfaces in the same package as the one in which they are used. [Source](https://docs.ros.org/en/foxy/Tutorials/Single-Package-Define-And-Use-Interface.html#link-against-the-interface)
