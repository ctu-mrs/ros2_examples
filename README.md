# ROS2 examples

This example package is meant to explore the possibilities of ROS2 from the point of view of current ROS1 features and how the ROS1 feature translate into the new ROS2 architecture.
We investigate the aspects that are currently utilized in [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system) with the intention of a potential future transition.

## Examples

Everything is a component. We happily [nodelet everything](https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Nodelet%20Everything.html) in ROS1, so why otherwise?

* [ ] [**ServiceClientExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_client_example.cpp) - periodically calls a service
  * [ ] TODO retrieving result synchronously. Some solutions are available, none work for me within a component
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor
* [ ] [**PublisherExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/publisher_example.cpp) - periodically publishes
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor
* [X] [**ServiceServerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_server_example.cpp) - getting called, ok
* [X] [**SubscriberExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/subscriber_example.cpp) - subscribes, ok
* [ ] [**TimerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/timer_example.cpp) - runs multiple timers in parallel
  * [ ] TODO timer callbacks are not executed in parallel even with multi-threaded container (with multi-threaded executor)
* [ ] [**ParamsExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/params_example.cpp) - load params from yaml and launch file
  * [X] param server callback is hooked up
  * [ ] TODO investigate the `ros__parameters:` namespace, which does not seem to be necessary (and does not work when present)

### Running the examples

* [tmuxinator](https://github.com/tmuxinator/tmuxinator) [session](https://github.com/ctu-mrs/ros2_examples/blob/master/tmux/session.yml)

`./tmux/start.sh` (change the sourced workspace path in `tmux/session.yml`)

## First ROS2 impressions

* [ ] **colcon**, the build system... is it really the best we have got? I want catkin back.
  * [ ] creates workspace wherever `colcon build` is called
  * [ ] therefore, cannot build in subdirectories
  * [ ] no `colcon clean`
  * [ ] overall not much user friendly
  * [X] immediately [aliased](https://github.com/ctu-mrs/uav_core/blob/281f16730f587200c29a1763379a08cd53d075d1/miscellaneous/shell_additions/shell_additions.sh#L475) it to fix those *improvements*
* [X] [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs) were ported to **ROS2**
* [ ] **ROS bridge** installs, compiles, runs ([scripts here](https://github.com/ctu-mrs/uav_core/tree/master/installation/ros2))
  * [ ] TODO check performance and load with images
  * [ ] TODO check performance and load high publish rates
* [ ] **Why are most examples still using `main()` when everything in ROS2 should be a component (nodelet)?**
  * [https://index.ros.org/doc/ros2/Tutorials/Composition/](https://index.ros.org/doc/ros2/Tutorials/Composition/)
  * `"Note: It is still possible to use the node-like style of “writing your own main” but for the common case it is not recommended."`
* [ ] **ComponentContainer**
  * [ ] **seems like the multi-threaded container (with multi-threaded executor) fails to execute callbacks (at least Timer callbacks) in parallel**
  * [ ] **multi-threaded composer (with multi-threaded executor) produces very uneven and slow rates of timers**, [Publishing is slow in Docker with MutliThreadedExecutor #1487](https://github.com/ros2/rclcpp/issues/1487) (December, 2020)
* [ ] **CMakeLists.txt**
  * [X] basics are fine
  * [ ] TODO building (and using) custom libraries
  * [ ] TODO building standalone executable nodelets (should be possible)
* [ ] **Launch files**
  * [X] they are in python now [link](https://index.ros.org/doc/ros2/Tutorials/Launch-Files/Creating-Launch-Files/)
  * [X] remapping, params, nodelets
  * [ ] TODO connecting to existing component containers (nodelet managers)
  * [ ] TODO launch prefixes (e.g., launching with GDB)
* [ ] **Subscribers** - work fine, not a big difference from ROS1
  * [ ] TODO still problem with multiple callbacks in parallel?
* [X] **Publishers** - work fine, not a big difference from ROS1
* [ ] **Service client**
  * [ ] it is asynchronous only!!!!
  * [ ] how to make it behave synchronously? [wrapper](https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/?answer=366458#post-id-366458) (only in standalone node)
* [X] **Service server** - works fine
* [ ] **Timers** - single timer works fine, similarly to ROS1 Timers
  * [ ] TODO still problem with multiple timers in parallel, recent (last active 2021/01/15) and related issues [Avoid timers to be executed twice in the multithreaded executor #1328](https://github.com/ros2/rclcpp/pull/1328) and [Allow timers to keep up the intended rate in MultiThreadedExecutor #1516](https://github.com/ros2/rclcpp/pull/1516) tell me that it is not settled how they should behave ?!?
* [ ] **Parameters**
  * [X] basic params work from *yaml* and *launch*
  * [X] **Beware!** loading an empty *yaml* file causes a long and cryptic error. **Solution:** add some random unused parameter.
  * [X] Nesting is distinguished by "." in the code (it was "/" in ROS1)
  * [ ] TODO test overloading multiple configs over each other
  * [ ] TODO test more complex types, array, matrices, ...
* [ ] **DRS** - has a direct implementation in the default **parameters**
  * [X] callback hooked to the external change of the parameters works
  * [ ] validation of the parameters has to be performed in our code -> we will need a custom wrapper for that, otherwise [**madness**](https://github.com/alsora/ros2-code-examples/blob/master/simple_parameter/src/simple_parameter_server_node.cpp)
  * [ ] ROS2 `rqt_reconfigure` is super buggy and wonky, this [issue](https://github.com/ros-visualization/rqt_reconfigure/issues/97) is especially hellish
    * [ ] it shows the node's parameters sometimes, seems random
* [X] why is the terminal output monochromatic? I want my colors back...
  * [X] `export RCUTILS_COLORIZED_OUTPUT=1`
* [ ] **ROS Time**
  * [ ] TODO duration, rate, sleep
* [ ] **Transformations**
  * [ ] TODO
* [ ] **Tests**
  * [ ] TODO
* [ ] **Pluginlib**
  * [ ] TODO
* [ ] **Actionlib**
  * [ ] TODO
* [ ] **Lifecycles** (new feature)
  * [ ] TODO
