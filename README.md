# ROS2 examples

## Examples

* [ ] **ServiceClientExample** - periodically calls a service
  * [ ] TODO retrieving result synchronously, more solutions available, none work for me within a component
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor
* [ ] **PublisherExample** - periodically publishes
  * [ ] TODO slow and irregular publishing (Timer-driven) with multi-threaded executor
* [X] **ServiceServerExample** - getting called
* [X] **SubscriberExample** - subscribes
* [ ] **TimerExample** - runs multiple timers in parallel
  * [ ] TODO timer callbacks are not executed in parallel even with multi-threaded container (with multi-threaded executor)
* [ ] **ParamsExample** - load params from yaml and launch file
  * [X] param server callback is hooked up
  * [ ] TODO nested params do not work
  * [ ] TODO investigate the `ros__parameters:` namespace, which does not seem to be neccessary

### Running examples

`./tmux/start.sh` (change the sourced workspace path in `tmux/session.yml`)

## First ROS2 impressions

* [X] [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs) were ported to **ROS2**
* [ ] **ROS bridge** installs, compiles, runs ([scripts here](https://github.com/ctu-mrs/uav_core/tree/master/installation/ros2))
  * [ ] TODO performance with images?
  * [ ] TODO performance high data rates?
* [ ] **Why are most examples on standalone node with `main()` when "everything in ROS2 should be a component (nodelet)?**
* [ ] **ComponentContainer**
  * [ ] seems like the multi-threaded container (with multi-threaded executor) fails to execute callbacks (at least Timer callbacks) in parallel
  * [ ] multi-threaded composer (with multi-threaded executor) produces very uneven and slow rates of timers
* [ ] **CMakeLists.txt**
  * [X] basics are fine
  * [ ] TODO building (and using) custom libraries
  * [ ] TODO building standalone executable nodelets (should be possible)
* [ ] **Launch files**
  * [X] they are in python
  * [X] remapping, params, nodelets
  * [ ] TODO launching and connecting to nodelet managers
  * [ ] TODO launch prefixes
* [X] **Subscribers** - work fine, not a big difference from ROS1
* [X] **Publishers** - work fine, not a big difference from ROS1
* [ ] **Service client**
  * [ ] it is asynchronous only!!!!
  * [ ] how to make it behave synchronously? [wrapper](https://answers.ros.org/question/343279/ros2-how-to-implement-a-sync-service-client-in-a-node/?answer=366458#post-id-366458) (only in standalone node)
* [X] **Service server** - works fine
* [X] **Timers** - work fine, not a big difference from ROS1
* [ ] **Parameters**
  * [X] basic params work from _yaml_ and _launch_
  * [ ] nested parameters do not work for me yet
* [ ] **DRS** - has a direct implementation in the default **parameters**
  * [X] callback hooked to the external change of the parameters works
  * [ ] validation of the parameters has to be performed in our code -> we will need a custom wrapper for that, otherwise [**madness**](https://github.com/alsora/ros2-code-examples/blob/master/simple_parameter/src/simple_parameter_server_node.cpp)
  * [ ] ROS2 rqt_reconfigure is super buggy and wonky
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
