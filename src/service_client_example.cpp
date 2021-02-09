#include <condition_variable>
#include <mutex>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{
  class ServiceClientExample : public rclcpp::Node
  {
    public:
      // | ----------------------- constructor ---------------------- |
      ServiceClientExample(rclcpp::NodeOptions options) : Node("service_client_example", options)
      {
        RCLCPP_INFO(get_logger(), "[ServiceClientExample]: initializing");

        m_service_client = create_client<std_srvs::srv::SetBool>("~/set_bool_out");

        m_main_thread = std::thread(&ServiceClientExample::main_thread, this);

        RCLCPP_INFO(get_logger(), "[ServiceClientExample]: initialized");
      }

      ~ServiceClientExample()
      {
        m_main_thread.join();
      }

    private:
      std::thread m_main_thread;
      rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr m_service_client;

    private:
      // | ------------------- main thread method ------------------- |
      void main_thread()
      {
        // do some config loading, preparation, wait for other nodes to become ready, etc.

        // wait for service to become available
        while (!m_service_client->wait_for_service(1s))
        {
          if (!rclcpp::ok())
          {
            RCLCPP_ERROR(get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
          }
          else
            RCLCPP_INFO(get_logger(), "service not available, waiting again...");
        }

        // call the service eg. to start some state-machine to control the robot
        const bool success = call_start_service();

        // check if call was successfull - if not, alert the user and abort
        if (success)
          RCLCPP_INFO(get_logger(), "[ServiceClientExample]: everything OK");
        else
          RCLCPP_ERROR(get_logger(), "[ServiceClientExample]: everything NOT OK! Failed to call service :(");
      }

      // | ---------------- call_start_service method --------------- |
      bool call_start_service()
      {
        RCLCPP_INFO(get_logger(), "[ServiceClientExample]: calling service");

        // THE BLOAT PARAGRAPH
        // helper type aliases
        // prepare the synchronization primitives
        std::mutex srv_mtx;
        std::condition_variable srv_cv;
        rclcpp::Client<std_srvs::srv::SetBool>::SharedResponse srv_resp;
        // prepare the callback lambda function
        const auto response_received_callback = [this, &srv_mtx, &srv_cv, &srv_resp](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture fut)
          {
            RCLCPP_INFO(get_logger(), "[ServiceClientExample]: service result received");
            {
              std::scoped_lock lck(srv_mtx);
              srv_resp = fut.get();
            }
            srv_cv.notify_all();
          };
        // remember to lock the mutex before the call
        std::unique_lock lck(srv_mtx);

        // THE ACTUALLY INTERESTING CODE
        // prepare the request
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true; // why no constructor :(
        // now finally actually call the service
        m_service_client->async_send_request(request, response_received_callback);  // with a callback, which will notify our condition variable

        // and wait for the response (MORE BLOAT!)
        while (rclcpp::ok())
        {
          const auto timeout = 1s;
          if (srv_cv.wait_for(lck, timeout) == std::cv_status::no_timeout)
            return request->data;
        }
        RCLCPP_INFO(get_logger(), "[ServiceClientExample]: got a ROS shutdown request, ending");
        return false;
      }
  };
}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceClientExample)
