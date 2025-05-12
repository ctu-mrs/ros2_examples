# WaypointFlier Simple - ROS2 simple example

This package was created as an example of how to write a very simple ROS2 node.
You can test the program in simulation run from `./tmux/start.sh`.

## Functionality

* Once activated, the node will command an UAV to fly through random waypoints
* Service `start_waypoint_following` will activate the node
* The area in which random waypoints are generated is configurable with a separate config file See [.yaml files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)

## How to start

```bash
./tmux/start.sh
```

Then, call the services prepared in the terminal window either by:

1. Pressing tmux binding (`Ctrl + b` or `Ctrl + a`)
2. Pressing the down arrow to change to the terminal below
3. Pressing the up arrow to bring up the prepared terminal command

Or typing the following command into a terminal connected to the ROS server:
```bash
ros2 service call /uav1/waypoint_flier_simple/start std_srvs/srv/Trigger
```

## Package structure

See [ROS2 packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html)

* `src` directory contains all source files
* `launch` directory contains `.py` launch files which are used to parametrize the node. Command-line arguments, as well as environment variables, can be loaded from the launch files, the node can be put into the correct namespace (each UAV has its namespace to allow multi-robot applications), config files are loaded, and parameters passed to the node. See [launch files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
* `config` directory contains parameters in `.yaml` files. See [.yaml files](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html)
* `package.xml` defines properties of the package, such as package name and dependencies. See [package.xml](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html#customize-package-xml)
* CMakeLists.txt contains instructions for building the package using the [colcon](https://colcon.readthedocs.io/en/released/user/quick-start.html) build tool

## More complex example

* To see a similar node with more functionality and features, see the `waypoint_flier` example.
