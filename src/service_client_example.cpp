#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{
  /* class ServiceClientExample //{ */

  class ServiceClientExample : public rclcpp::Node
  {
  public:
    ServiceClientExample(const rclcpp::NodeOptions & options);

  private:
    // | --------------------- service clients -------------------- |

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;


    // | ------------------------- timers ------------------------- |

    rclcpp::TimerBase::SharedPtr timer_async_;
    rclcpp::TimerBase::SharedPtr timer_sync_;

    void callback_timer_async();

    void callback_timer_sync();
  };

  //}

  /* ServiceClientExample() //{ */

  ServiceClientExample::ServiceClientExample(const rclcpp::NodeOptions & options) : Node("service_client_example", options)
  {
    RCLCPP_INFO_STREAM(get_logger(), "initializing");

    // | --------------------- service client --------------------- |

    service_client_ = create_client<std_srvs::srv::SetBool>("~/out_set_bool");

    // | -------------------------- timer ------------------------- |

    // There are two ways you can check the result of the service call:
    // 1. using std::future<Result>
    // 2. using a callback
    // Examples for each case follow.

    // both the timers are going to be run inside the same thread as this example will be run inside a Single Threaded Executor
    timer_async_ = create_wall_timer(std::chrono::duration<double>(2.0 / 1.0), std::bind(&ServiceClientExample::callback_timer_async, this));
    /* timer_sync_ = create_wall_timer(std::chrono::duration<double>(2.0 / 1.0), std::bind(&ServiceClientExample::callback_timer_sync, this)); */

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO_STREAM(get_logger(), "initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* timer_future() //{ */

  // a timer callback should never block for too long, so all checks should return after a while
  void ServiceClientExample::callback_timer_sync()
  {
    // if the future is already waiting for a result, check if we got one
    /* if (future_setbool_.valid()) */
    /* { */
      if (service_client_->service_is_ready())
      {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;

        const auto future_setbool = service_client_->async_send_request(request).future.share();
        RCLCPP_INFO_STREAM(get_logger(), "Service called in sync mode. Timer will wait for the response.");

        if (future_setbool.valid() == false)
        {
          RCLCPP_WARN_STREAM(get_logger(), "Future object was in undefined state.");
          return;
        }

        /* const auto status = future_setbool.wait_for(std::chrono::seconds(4)); */
        /* if (status == std::future_status::ready) */
        /* { */
          // future already got a result, check it and report to user
          const auto result = future_setbool.get();
          if (result->success)
          {
            RCLCPP_INFO_STREAM(get_logger(), "Service call successful with message: " << result->message);
          }
          else
          {
            RCLCPP_INFO_STREAM(get_logger(), "Service call reports failure: " << result->message);
          }
        /* } */
        /* else */
        /* { */
        /*   RCLCPP_WARN_STREAM(get_logger(), "Timeout reached while waiting for service response!"); */
        /* } */
      }
      else
      {
        RCLCPP_WARN_STREAM(get_logger(), "Can not call service in sync mode. Service not ready!");
      }

    /* } */
    // if the future is not waiting for a result, make a new service call
    /* else */
    /* { */
    /*   // you should not send a service request unless the service is ready, otherwise the future will never be ready */
    /*   if (service_client_->service_is_ready()) */
    /*   { */
    /*     auto request = std::make_shared<std_srvs::srv::SetBool::Request>(); */
    /*     request->data = true; */
    /*     future_setbool_ = service_client_->async_send_request(request).future.share(); */
    /*     RCLCPP_INFO_STREAM(get_logger(), "called service using future"); */
    /*   } */
    /*   else */
    /*   { */
    /*     RCLCPP_WARN_STREAM(get_logger(), "not calling service using future, service not ready!"); */
    /*   } */
    /* } */
  }

  //}

  // | ------------------------ routines ------------------------ |



  /* timer_callback() //{ */

  void ServiceClientExample::callback_timer_async()
  {
    // it's good to first check if the service server is even ready to be called
    if (service_client_->service_is_ready())
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      service_client_->async_send_request(request, [&](const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture fut)
        // The callback function for service call response.
        // This function will be executed once the other side decides to respond to the service call.
        {
          // the future must be ready now that the callback was called
          const auto result = fut.get();
          if (result->success)
          {
            RCLCPP_INFO_STREAM(get_logger(), "Async callback called, service call successful with message: " << result->message);
          }
          else
          {
            RCLCPP_INFO_STREAM(get_logger(), "Async callback called, service call reports failure: " << result->message);
          }
        }
        );

      RCLCPP_INFO_STREAM(get_logger(), "Service called in async mode with a callback. Timer will not wait.");
    }
    else
    {
      RCLCPP_INFO_STREAM(get_logger(), "Can not call service in async mode. Service not ready!");
    }
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceClientExample)
