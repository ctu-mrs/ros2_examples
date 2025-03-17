# MRS ROS C++ Vision example

This package was created as an example of how to use OpenCV and write packages for image processing or computer vision.
For a more general package example, see the [waypoint_flier](https://github.com/ctu-mrs/mrs_core_examples/tree/master/cpp/waypoint_flier).
You can test the program in simulation (see our [simulation tutorial](https://ctu-mrs.github.io/docs/simulation/howto.html)).

## Example features

* Subscribing to camera topic (using ImageTransport, which is agnostic to the image compression etc.)
* Publishing images (again using ImageTransport, enabling automatic compression of the output)
* Basic OpenCV image processing (Canny edge detection, image blurring etc.)
* Basic OpenCV drawing functions
* Using TF2 to transform points between frames
* Backprojection of 3D points to the camera image
* Other features, which overlap with the [waypoint_flier](https://github.com/ctu-mrs/mrs_core_examples/tree/master/cpp/waypoint_flier) template

## How to start it?

```bash
./tmux/start.sh
```

## Coding practices

Coding practices, related to packages working with images, are described here.
For a more detailed description of good programming practices in the context of ROS, see the [waypoint_flier](https://github.com/ctu-mrs/mrs_core_examples/tree/master/cpp/waypoint_flier).
Also check out our general [C++ good/bad coding practices tutorial](https://ctu-mrs.github.io/docs/introduction/c_to_cpp.html).

### Using `cv_bridge::toCvShare()` or `cv_bridge::toCvCopy()` for converting between `sensor_msgs::Image` and `cv::Mat`
*Reference documentation: [C++ API docs](http://docs.ros.org/melodic/api/cv_bridge/html/c++/), [example usage](https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages)*

Use `cv_bridge::toCvShare()` to avoid copying input images, if applicable.
In contrast to `cv_bridge::toCvCopy()`, which allows modifying the returned data, `cv_bridge::toCvShare()` returns a `cv_bridge::CvImageConstPtr` pointing to the image data, which may be shared by all nodes/nodelets subscribing the image topic.
This is why you must *avoid modifying the image, returned by `cv_bridge::toCvShare()`*!

The rule of thumb whether to use `cv_bridge::toCvCopy()` or `cv_bridge::toCvShare()` can be summarized as:

* If you plan on modifying the image data (such as drawing to the image, blurring it, applying erosion etc.), either use `cv_bridge::toCvShare()` and then `cv::Mat::copyTo()` the returned OpenCV matrix to one you are going to modify, or simply use `cv_bridge::toCvCopy()`.
* If you don't want to modify the image data (for example when you only want to display it or if you want to use e.g. `cv::cvtColor()` to convert a color image to grayscale), use `cv_bridge::toCvShare()` to avoid unnecessary copies.

When using either `cv_bridge::toCvShare()` or `cv_bridge::toCvCopy()`, it is a good practice to specify the desired encoding to which the data should be converted if it comes in a different encoding.
A typical example why this is a good idea is when you plan on using an image from a color camera, which uses RGB ordering of the data, in OpenCV, which mostly presumes BGR ordering.
If you pass `"bgr8"` to `cv_bridge::toCvShare()` or `cv_bridge::toCvCopy()`, and the image data in the ROS message use RGB ordering, the function will automatically reorder the data to BGR.

For more information, see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages.

### Using `image_geometry::PinholeCameraModel` for 2D->3D and 3D->2D transformation of points
*Reference documentation: [`image_geometry::PinholeCameraModel`](http://docs.ros.org/api/image_geometry/html/c++/classimage__geometry_1_1PinholeCameraModel.html), [camera_calibration](https://wiki.ros.org/camera_calibration)*

If you want to project a 2D position in the image to a 3D ray or backproject a 3D point to the image, the most straightforward way is using the pinhole camera model, which is conveniently implemented in the ROS `image_geometry` package as the class `image_geometry::PinholeCameraModel`.
The pinhole model works reasonably well for standard cameras, although if you need to use a more exotic camera type (such as an omnidirectional camera), it might not be sufficient (in that case, refer to the [OCamCalib toolbox](https://sites.google.com/site/scarabotix/ocamcalib-toolbox)).
To use the pinhole model, distortion of the camera has to be compensated for.
This is done using the `image_geometry::PinholeCameraModel::rectifyPoint()` and `image_geometry::PinholeCameraModel::unrectifyPoint()` methods.
Note that the terminology used in the `image_geometry::PinholeCameraModel` class is not precise, since technically image undistortion and rectification are different things and in this case, the methods `rectifyPoint` and `unrectifyPoint` are doing undistortion and distortion of the 2D pixel coordinates, not rectification.
The camera parameters need to be known to initialize the `image_geometry::PinholeCameraModel` object.
These are mostly obtained using offline calibration and the convenient ROS calibration tool can be used for this by running `rosrun camera_calibration cameracalibrator.py`.
Some cameras (such as cameras from the Realsense family) publish already undistorted images.
The undistort/distort steps can then be skipped, but it's usually better practice to leave them in in case a different camera was used.

**Using `image_geometry::PinholeCameraModel` to project a 2D point to 3D has more or less the following steps:**

1) Obtain parameters of the camera to be used. Usually, a camera will publish these parameters on the corresponding `camera_info` topic.
2) Initialize the `image_geometry::PinholeCameraModel` using the parameters from step 1.
3) Undistort the point coordinates. Use the `image_geometry::PinholeCameraModel::rectifyPoint()` method.
4) Project the undistorted point coordinates to 3D. Use `image_geometry::PinholeCameraModel::projectPixelTo3dRay()`.

Note that the result is a 3D vector in the camera optical coordinate frame.
To get a 3D point, you need to somehow estimate the distance of the point from the camera center.
It may also be necessary to transform the point to other coordinate frame (see the next section for how to use the ROS TF2 framework).

**Using `image_geometry::PinholeCameraModel` to backproject a 3D point to the image (this is demonstrated in the `EdgeDetect` nodelet):**

1) Obtain parameters of the camera to be used. Usually, a camera will publish these parameters on the corresponding `camera_info` topic.
2) Initialize the `image_geometry::PinholeCameraModel` using the parameters from step 1.
2) Transform the 3D point to the camera optical coordinate frame. Use the ROS TF2 framework as described in the next section.
3) Backproject the point to 2D. Use the `image_geometry::PinholeCameraModel::project3dToPixel()` method.
4) Apply camera distortion. Use the `image_geometry::PinholeCameraModel::unrectifyPoint()` method.

### Using the ROS TF2 framework for transforming stuff between different coordinate frames

*Reference documentation: [TF2 docs](http://wiki.ros.org/tf2_ros), [tutorial](http://wiki.ros.org/tf2/Tutorials/Introduction%20to%20tf2)*

First off, you can use `rosrun rqt_tf_tree rqt_tf_tree` or `rosrun tf view_frames && zathura frames.pdf && rm frames.pdf && rm frames.gv` (the first one is recommended) to inspect frames, which are currently available in `roscore`, and their mutual connections.
To observe a transformation between two specific frames in real-time, use `rosrun tf tf_echo [reference_frame] [target_frame]`.
By restarting `roscore`, the frames will reset.
Sometimes, RViz will be stubborn and refuse to use frames, which exist and are correctly connected (you can check that with the abovementioned command).
The solution is usually clicking the `Reset` button in RViz or restarting it.

If you need to publish a static transformation between two frames (typically from the UAV frame to the camera optical frame), you can use `rosrun tf2_ros static_transform_publisher dx dy dz yaw pitch roll frame_id child_frame_id`, where `frame_id` would typically be the UAV frame (i.e. `fcu_uavX`) and  `child_frame_id` would be e.g. the camera frame.

To use transformations from your code, you need to have a `tf2_ros::Buffer` object, which buffers the transformations and provides lookups between frames at specific time-stamps.
Aditionally, you need a `tf2_ros::TransformListener`, which listens to the transformation messages and fills the `tf2_ros::Buffer`.
Because the `tf2_ros::TransformListener` class does not implement the assignment operator and it cannot be initialized in the constructor of a nodelet, the usual practice is to have a smart-pointer member variable (i.e. `std::unique_ptr<tf2_ros::TransformListener>`) and instantiate the object in the `OnInit()` method of the nodelet class using `std::make_unique<tf2_ros::TransformListener>()`.

To actually do the transformation, you need to follow these steps:

1) Try to lookup the transform using `tf2_ros::Buffer::lookupTransform()` and a `try` block.
2) Catch a potential `tf2::TransformException` exception using a `catch` block. If exception is caught, alert the user and abort the transformation process.
3) If the lookup was successful, do the actual transformation using `tf2::doTransform()`.

It is important to note that to use the `tf2::doTransform()` method, you must add the corresponding lib to the `CMakeLists.txt` and `package.xml` files.
For example if you want to transform messages from the `geometry_msgs` package, you need to include `tf2_geometry_msgs` in the `find_package()` function in the `CMakeLists.txt` file and as a dependency in the `package.xml` file, and include the `<tf2_geometry_msgs/tf2_geometry_msgs.h>` header in the corresponding C++ file.
