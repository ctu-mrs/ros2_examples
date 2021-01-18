# ROS2 examples

This example package is meant to explore the possibilities of ROS2 from the point of view of current ROS1 features and how the ROS1 feature translate into the new ROS2 architecture.

## Examples

* [ ] [**ServiceClientExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_client_example.cpp) - periodically calls a service
  * [ ] TODO retrieving result synchronously, more solutions available, none work for me within a component
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor
* [ ] [**PublisherExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/publisher_example.cpp) - periodically publishes
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor
* [X] [**ServiceServerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/service_server_example.cpp) - getting called, ok
* [X] [**SubscriberExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/subscriber_example.cpp) - subscribes, ok
* [ ] [**TimerExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/timer_example.cpp) - runs multiple timers in parallel
  * [ ] TODO timer callbacks are not executed in parallel even with multi-threaded container (with multi-threaded executor)
* [ ] [**ParamsExample**](https://github.com/ctu-mrs/ros2_examples/blob/master/src/params_example.cpp) - load params from yaml and launch file
  * [X] param server callback is hooked up
  * [ ] TODO nested params do not work
  * [ ] TODO investigate the `ros__parameters:` namespace, which does not seem to be neccessary

### Running the examples

* [tmuxinator](https://github.com/tmuxinator/tmuxinator) [session](https://github.com/ctu-mrs/ros2_examples/blob/master/tmux/session.yml)

`./tmux/start.sh` (change the sourced workspace path in `tmux/session.yml`)

## First ROS2 impressions

* [X] [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs) were ported to **ROS2**
* [ ] **ROS bridge** installs, compiles, runs ([scripts here](https://github.com/ctu-mrs/uav_core/tree/master/installation/ros2))
  * [ ] TODO check performance and load with images
  * [ ] TODO check performance and load high publish rates
* [ ] **Why are most examples on still use `main()` when everything in ROS2 should be a component (nodelet)?**
  * [https://index.ros.org/doc/ros2/Tutorials/Composition/](https://index.ros.org/doc/ros2/Tutorials/Composition/)
  * `"Note: It is still possible to use the node-like style of “writing your own main” but for the common case it is not recommended."`
* [ ] **ComponentContainer**
  * [ ] **seems like the multi-threaded container (with multi-threaded executor) fails to execute callbacks (at least Timer callbacks) in parallel**
  * [ ] **multi-threaded composer (with multi-threaded executor) produces very uneven and slow rates of timers**
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
  * [ ] TODO still problem with multiple timers in parallel
* [ ] **Parameters**
  * [X] basic params work from _yaml_ and _launch_
  * [ ] TODO fix nested parameters
* [ ] **DRS** - has a direct implementation in the default **parameters**
  * [X] callback hooked to the external change of the parameters works
  * [ ] validation of the parameters has to be performed in our code -> we will need a custom wrapper for that, otherwise [**madness**](https://github.com/alsora/ros2-code-examples/blob/master/simple_parameter/src/simple_parameter_server_node.cpp)
  * [ ] ROS2 rqt_reconfigure is super buggy and wonky, this [issue](https://github.com/ros-visualization/rqt_reconfigure/issues/97) is specially hellish
* [X] why is the terminal output monochromatic? I want my colors back...
  * [X] `export RCUTILS_COLORIZED_OUTPUT=1`
* [ ] **ROS Time**
  * [ ] TODO duration, rate, sleep
* [ ] **Transformations**
  * [ ] TODO
* [ ] **Pluginlib**
  * [ ] TODO
* [ ] **Lifecycles** (new feature)
  * [ ] TODO
