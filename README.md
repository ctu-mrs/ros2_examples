# ROS2 examples

This example package is meant to explore the possibilities of ROS2 from the point of view of current ROS1 features and how the ROS1 feature translate into the new ROS2 architecture.
We investigate the aspects that are currently utilized in [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) with the intention of a potential future transition.

**Meant to be tested on 20.04 Foxy**

## Minimalistic Examples

Everything is a component. We happily [nodelet everything](https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Nodelet%20Everything.html) in ROS1, so why otherwise?

* [ ] [**ServiceClientExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_client_example.cpp) - periodically calls a service
  * [ ] TODO retrieving result synchronously. Some solutions are available, none work for me within a component
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor (looks like the same problem can appear even with single-threaded executor)
* [ ] [**PublisherExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/publisher_example.cpp) - periodically publishes
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor (looks like the same problem can appear even with single-threaded executor)
* [X] [**ServiceServerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_server_example.cpp) - getting called, ok
* [X] [**SubscriberExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/subscriber_example.cpp) - subscribes, ok
* [X] [**TimerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/timer_example.cpp) - runs multiple timers in parallel
  * [X] ~~timer callbacks are not executed in parallel even with multi-threaded container (with multi-threaded executor, looks like the same problem can appear even with single-threaded executor)~~ --> timers were constructed as part of *mutual exclusive callbackGroup*
* [ ] [**ParamsExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/params_example.cpp) - load params from yaml and launch file
  * [X] param server callback is hooked up
  * [ ] TODO investigate the `ros__parameters:` namespace, which does not seem to be necessary (and does not work when present)
* [X] [**Tf2BroadcasterExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/tf2_broadcaster_example.cpp) - publishing transforms, ok
  * [ ] TODO no innovation in TF structure, missing the option to have multiple parents
* [X] [**Tf2ListenerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/tf2_listener_example.cpp) - receiving transforms, lookup table works, ok

### Running the examples

* [tmuxinator](https://github.com/tmuxinator/tmuxinator) [session](https://github.com/ctu-mrs/ros2_examples/blob/master/tmux/session.yml)

`./tmux/start.sh` (change the sourced workspace path in `tmux/session.yml`)

## First ROS2 impressions

* [ ] **colcon**, the build system... is it really the best we have got? I want my catkin back.
  * creates workspace wherever `colcon build` is called
  * therefore, cannot build in subdirectories
  * no `colcon clean`, no `colcon init`
  * overall not much user friendly
  * immediately [aliased](https://github.com/ctu-mrs/uav_core/blob/281f16730f587200c29a1763379a08cd53d075d1/miscellaneous/shell_additions/shell_additions.sh#L475) it to fix those *hurdles*
  * [ ] TODO workspace-wide profiles with custom flags
    * profile are not supported by colcon yet. See [discussion](https://github.com/colcon/colcon-core/issues/168) 
    * predefining of build setting with custom flags can be done using mixin
      * install: `sudo apt install python3-colcon-mixin`
      * REPO with readme and mixin examples: [repo](https://github.com/colcon/colcon-mixin-repository) 
      * example of usage: `colcon build --mixin rel-with-deb-info`
* [X] Sourcing ROS2 workspace
  * ROS2 sourcing
```bash
  source /opt/ros/foxy/setup.zsh
  source ~/ros2_bridge_workspace/install/setup.zsh
  source ~/ros2_workspace/install/setup.zsh
```
  * if you build ROS2 workspace while ROS1 is sourced, you will need to source ROS1 every time before launching ROS2 programs, otherwise, this error will appear:
```
[INFO] [launch]: All log files can be found below /home/klaxalk/.ros/log/2021-02-16-09-29-27-258633-klaxalk-desktop2-1671935
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [component_container_mt-1]: process started with pid [1671949]
[component_container_mt-1] [INFO] [1613464167.606666336] [nmspc1_timer_example]: Load Library: /home/klaxalk/ros2_workspace/install/ros2_examples/lib/libtimer_example.so
[component_container_mt-1] /opt/ros/foxy/lib/rclcpp_components/component_container_mt: symbol lookup error: /home/klaxalk/ros2_workspace/install/ros2_examples/lib/libtimer_example.so: undefined symbol: _ZN12class_loader4impl22AbstractMetaObjectBaseC2ERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEES9_S9_
[ERROR] [component_container_mt-1]: process has died [pid 1671949, exit code 127, cmd '/opt/ros/foxy/lib/rclcpp_components/component_container_mt --ros-args -r __node:=nmspc1_timer_example -r __ns:=/'].
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT) again, ignoring...
```
* [ ] *Topic* the subscribers and publisher have to have compatible Quality of Service (QoS) settings -- reliable or best effort. For example RViZ is setting different profile (reliable) that is by default (best effort). [QoS](https://index.ros.org/doc/ros2/Concepts/About-Quality-of-Service-Settings/)  
* [X] [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs/tree/ros2) were ported to **ROS2**
* [ ] **ROS bridge** installs, compiles, runs ([scripts here](https://github.com/ctu-mrs/uav_core/tree/master/installation/ros2))
  * [ ] TODO check performance and load with images
  * [ ] TODO check performance and load high publish rates
* [ ] **Why are most examples still using `main()` when everything in ROS2 should be a component (nodelet)?**
  * [https://index.ros.org/doc/ros2/Tutorials/Composition/](https://index.ros.org/doc/ros2/Tutorials/Composition/)
  * `"Note: It is still possible to use the node-like style of “writing your own main” but for the common case it is not recommended."`
* [ ] **ComponentContainer**
  * [ ] try running a component as single node by:
```
  remap = [('cloud_in', '/livox/lidar')]
  node = Node(package='octomap_server2',
               executable='octomap_server',
               output='screen',
               remappings=remap,
               parameters=[params])
  return LaunchDescription([node])
```
  * [X] **seems like the multi-threaded container (with multi-threaded executor) fails to execute callbacks (at least Timer callbacks) in parallel** --> the rclcpp::callback_group::CallbackGroupType::Reentrant has to be selected for parallel execution of callback driven classes (subscribers and timers)
  * [X] **multi-threaded composer (with multi-threaded executor) produces very uneven and slow rates of timers**, [Allow timers to keep up the intended rate in MultiThreadedExecutor #1516](https://github.com/ros2/rclcpp/pull/1516) -> merged already and available for version rclcpp 8.0 and higher
* [ ] **CMakeLists.txt**
  * [X] basics are fine
  * [X] building (and using) custom libraries
  * [X] building standalone executable nodelets
* [ ] **Launch files**
  * [X] they are in python now [link](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/)
  * [X] remapping, params, nodelets
  * [ ] TODO connecting to existing component containers (nodelet managers)
  * [X] launch prefixes (e.g., launching with GDB) [link](https://github.com/ctu-mrs/mrs_lib/blob/ros2/launch/params_example.py)
* [X] **Subscribers** - work fine, just be aware of difference between CallbackGroupType::Reentrant and CallbackGroupType::MutuallyExclusive (default one).
* [X] **Publishers** - works fine, not a big difference from ROS1
* [ ] **Service client**
  * [ ] it is asynchronous only!!!!
  * [ ] how to make it behave synchronously? [wrapper](https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/?answer=366458#post-id-366458) (only in standalone node)
* [X] **Service server** - works fine
* [X] **Timers** - works fine, just be aware of difference between CallbackGroupType::Reentrant and CallbackGroupType::MutuallyExclusive (default one).
* [ ] **Parameters**
  * [X] basic params work from *yaml* and *launch*
  * [X] **Beware!** loading an empty *yaml* file causes a long and cryptic error. **Solution:** add some unused parameter to the file.
  * [X] Nesting is distinguished by "." in the code (it was "/" in ROS1)
  * [ ] TODO test overloading multiple configs over each other
  * [ ] TODO test more complex types: lists, matrices, ...
* [ ] **DRS** - has a direct implementation in the default **parameters**
  * [X] callback hooked to the external change of the parameters works
  * [ ] validation of the parameters has to be performed in our code -> we will need a custom wrapper for that, otherwise [**madness**](https://github.com/alsora/ros2-code-examples/blob/master/simple_parameter/src/simple_parameter_server_node.cpp)
  * [ ] ROS2 `rqt_reconfigure` is super buggy and wonky, this [issue](https://github.com/ros-visualization/rqt_reconfigure/issues/97) is especially hellish
    * [ ] it shows the node's parameters sometimes, seems random
* [X] why is the terminal output monochromatic? I want my colors back...
  * [X] `export RCUTILS_COLORIZED_OUTPUT=1`
* [ ] **ROS Time**
  * [ ] TODO test duration, rate, sleep, wall time vs. sim time (we need sim time for faster/slower than real time simulations)
    * [X]  To be able to listen sim time, parameter use_sime_time=1 has to be set individually for every node. [PR](https://github.com/ros2/rclcpp/pull/559)
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
* I had to use [eigen3_cmake_module](https://github.com/ros2/eigen3_cmake_module) to successfully force ament_cmake to find eigen during building process.
### Messages/Services
* In order to use the messages generated in the same package we need to use the following CMake code:
  ```cmake
  rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
  ```
  This finds the relevant generated C++ code from ``AddressBook.msg`` and allows your target to link against it.

  You may have noticed that this step was not necessary when the interfaces being used were from a package that was built separately. This CMake code is only     required when you want to use interfaces in the same package as the one in which they are used. [Source](https://docs.ros.org/en/foxy/Tutorials/Single-Package-Define-And-Use-Interface.html#link-against-the-interface)
