# ros2_uav_example

## First impressions

* [X] **ROS bridge**
* [X] **ROS2 mrs_msgs**
* [.] **CMakeLists.txt**
  * [X] basics are fine
  * [ ] TODO building (and using) custom libraries
  * [ ] TODO building standalone executable nodelets (should be possible)
* [o] **Launch files**
  * [X] they are in python
  * [X] remapping, params, nodelets
  * [ ] TODO launching and connecting to nodelet managers
  * [ ] TODO launch prefixes
* [X] **Subscribers** - work fine, not a big difference from ROS1
* [X] **Publishers** - work fine, not a big difference from ROS1
* [X] **Timers** - work fine, not a big difference from ROS1
* [X] **Parameters**
  * [X] somehow work from _yaml_ and _launch_
* [.] **DRS** - has a direct implementation in the default **parameters**
  * [X] callback hooked to the external change of the parameters works
  * [ ] validation of the parameters has to be performed in our code -> we will need a custom wrapper for that, otherwise madness
  * [ ] ROS2 rqt_reconfigure is super buggy and wonky
* [ ] why is the terminal output monochromatic? I want my colors back...
* [ ] **ROS Time**
  * [ ] TODO
* [ ] **Transformetions**
  * [ ] TODO
* [ ] **Service client**
  * [ ] TODO
* [ ] **Service server**
  * [ ] TODO
* [ ] **Lifecycles**
  * [ ] TODO
* [ ] **Pluginlib**
  * [ ] TODO
