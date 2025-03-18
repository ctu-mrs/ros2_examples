#include <rclcpp/rclcpp.hpp>

/* camera image messages */
#include <sensor_msgs/msg/image.hpp>

/* some STL includes */
#include <stdlib.h>
#include <mutex>

/* some OpenCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

namespace vision_examples
{

class ExampleImageTransport : public rclcpp::Node {

public:
  ExampleImageTransport (const rclcpp::NodeOptions options);

private:
  std::atomic<bool> is_initialized_ = false;

  std::mutex mutex_image_;           // to prevent data races when accessing the following variables from multiple threads
  sensor_msgs::msg::Image::ConstSharedPtr image_;
  rclcpp::Time  time_last_image_;          // time stamp of the last received image message

  void                        callback_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  image_transport::Subscriber sub_image_;

  void       callback_timer();
  rclcpp::TimerBase::SharedPtr     timer_publisher_;

  image_transport::Publisher pub_edited_image_;
};

ExampleImageTransport::ExampleImageTransport(rclcpp::NodeOptions options) : Node("example_image_transport", options) {

  RCLCPP_INFO(get_logger(), "Initializing");

  {
    std::scoped_lock lck(mutex_image_);
    if(image_) {
      RCLCPP_ERROR(get_logger(), "Image pointer is assigned before initialization, shutting down");
      rclcpp::shutdown();
      return;
    }
  }

  auto sub_node = this->create_sub_node("image_transport");
  image_transport::ImageTransport image_transporter(sub_node);
  image_transport::TransportHints hints(sub_node.get());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  sub_image_       = image_transporter.subscribe("~/image_in", 1, &ExampleImageTransport::callback_image, this, &hints, sub_options);

  pub_edited_image_ = image_transporter.advertise("edited_image_out", 1);

  timer_publisher_ = create_timer(std::chrono::duration<double>(1.0 / 2.0), std::bind(&ExampleImageTransport::callback_timer, this));

  is_initialized_ = true;
  RCLCPP_INFO(get_logger(), "Initialized");
}

void ExampleImageTransport::callback_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {

  if (!is_initialized_) {
    return;
  }

  {
    std::scoped_lock lock(mutex_image_);
    time_last_image_ = this->now();
    image_ = msg;
  }
  RCLCPP_WARN(get_logger(), "Recieved new image");
}

void ExampleImageTransport::callback_timer() {

  if (!is_initialized_) {
    return;
  }

  sensor_msgs::msg::Image::ConstSharedPtr ptr_image;
  {
    std::scoped_lock lock(mutex_image_);

    ptr_image = image_;
  }

  if (ptr_image.use_count() == 0) {
    RCLCPP_WARN(get_logger(), "Waiting for 'image' to publish the edited image");
    return;
  }

  // toCvShare avoids copying the image data and instead copies only the (smart) constpointer
  // to the data. Then, the data cannot be changed (it is potentially shared between multiple nodes) and
  // it is automatically freed when all pointers to it are released. If you want to modify the image data,
  // use toCvCopy (see https://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages),
  // or copy the image data using cv::Mat::copyTo() method.
  // Adittionally, toCvShare and toCvCopy will convert the input image to the specified encoding
  // if it differs from the one in the message. Try to be consistent in what encodings you use throughout the code.
  cv_bridge::CvImagePtr ptr_cv_image = cv_bridge::toCvCopy(ptr_image, "bgr8");

  cv::putText(ptr_cv_image->image, std::to_string(this->now().seconds()), cv::Point(ptr_cv_image->image.cols / 2.0, ptr_cv_image->image.rows / 2.0), cv::FONT_HERSHEY_TRIPLEX, 1.0, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

  pub_edited_image_.publish(ptr_cv_image->toImageMsg());
}
}  // namespace vision_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(vision_examples::ExampleImageTransport)
