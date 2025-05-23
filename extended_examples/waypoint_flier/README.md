# WaypointFlier ROS example

This package was created as an example of how to write ROS nodelets.
The package is written in C++ and features custom MRS libraries and msgs.

## Functionality

* Desired waypoints are loaded as a matrix from config file
* Service `fly_to_first_waypoint` prepares the UAV by flying to the first waypoint
* Service `start_waypoint_following` causes the UAV to start tracking the waypoints
* Service `stop_waypoint_following` stops adding new waypoints. Flight to the current waypoint is not interrupted.

## How to start

```bash
./tmux/start.sh
```

Then, call the services prepared in the terminal window either by:

1. Pressing tmux binding (`Ctrl + b` or `Ctrl + a`)
2. Pressing the down arrow to change to the terminal below
3. Pressing the up arrow to bring up the prepared terminal command

Or typing the following command into a terminal connected to the ROS server:
```
ros2 service call /uav1/waypoint_flier/start_waypoints_following std_srvs/srv/Trigger
```

## Package structure

See [ROS2 packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

* `src` directory contains all source files
* `launch` directory contains `.py` launch files which are used to parametrize the node. Command-line arguments, as well as environment variables, can be loaded from the launch files, the node can be put into the correct namespace (each UAV has its namespace to allow multi-robot applications), config files are loaded, and parameters passed to the node. See [launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
* `config` directory contains parameters in `.yaml` files. See [.yaml files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml)
* CMakeLists.txt contains instructions for building the package using the [colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html) build tool

## Example features

Some of the following features are wrapped in `mrs_lib` interface to provide added functionalities, improved performance, and ease of use. Therefore, their interface can differ from the original ROS2 features.

* [Node](https://docs.ros.org/en/jazzy/Concepts/Basic/About-Nodes.html#nodes) initialization
* [Subscriber, publisher](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html), and [timer](https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1TimerBase.html) initialization
* [Service servers and clients](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html) initialization
* Loading [parameters](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-CPP.html) with `mrs_lib::ParamLoader` class
* Loading [Eigen matrices](https://eigen.tuxfamily.org/dox/group__TutorialMatrixClass.html) with `mrs_lib::ParamLoader` class
* Checking node initialization status in every callback
* Checking whether subscribed messages are coming
* Throttling [text output](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html) to a terminal
* [Thread-safe access](https://en.cppreference.com/w/cpp/thread/mutex) to variables using `std::scoped_lock`
* Using `ConstPtr` when subscribing to a topic to avoid copying large messages
* Storing and accessing matrices in `Eigen` classes
* [Remapping topics](http://wiki.ros.org/roslaunch/XML/remap) in the launch file

## Coding style

For easy orientation in the code, we have agreed to follow the [ROS C++ Style Guide](http://wiki.ros.org/CppStyleGuide) when writing our packages.
Also check out our general [C++ good/bad coding practices tutorial](https://ctu-mrs.github.io/docs/introduction/c_to_cpp.html).

### Naming variables

* Member variables are distinguished from local variables by underscore at the end:
  - `position_x` -  local variable
  - `position_x_` -  member variable
* Also, we distinguish parameters which are loaded as parameters by underscore at the beginning
* Descriptive variable names are used. The purpose of the variable should be obvious from the name.
  - `sub_odom_uav_` - member subscriber to uav odometry msg type
  - `pub_reference_` - member publisher of reference msg type
  - `srv_server_start_waypoints_following_` - member service server for starting following of waypoints
  - `ExampleWaypointFlier::callbackTimerCheckSubscribers()` - callback of timer which checks subscribers
  - `mutex_odom_uav_` - mutex locking access to variable containing odometry of the UAV

### Good practices

* Do not use raw pointers! Smart pointers from `<memory>` free resources automatically, thus preventing memory leaks.
* Lock access to member variables! Nodelets are multi-thread processes, so it is our responsibility to make our code thread-safe.
  - Use `c++17` `scoped_lock` which unlocks the mutex after leaving the scope. This way, you can't forget to unlock the mutex.
  ```cpp
  {
    std::scoped_lock lock(mutex_odom_uav_);
    odom_uav_ = *msg;
  }
  ```
* Use `mrs_lib::ParamLoader` class to load parameters from launch files and config files. This class checks whether the parameter was actually loaded, which can save a lot of debugging. Furthermore, loading matrices into config files becomes much simpler.
* For printing debug info to terminal use `RCLCPP_INFO()`, `RCLCPP_WARN()`, `RCLCPP_ERROR()` macros. Do not spam the terminal by printing a variable every time a callback is called, use for example `RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1.0, "dog")` to print *dog* not more often than every second. Other animals can also be used for debugging purposes.
* If you need to execute a piece of code periodically, do not use sleep in a loop, or anything similar. The ROS API provides `rclcpp::TimerBase` class for this purposes, which executes a callback every time the timer expires.
* Always check whether all subscribed messages are coming. If not, print a warning. Then you know the problem is not in your nodelet and you know to look for the problem in topic remapping or the node publishing it.
