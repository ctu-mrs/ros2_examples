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

    rclcpp::TimerBase::SharedPtr timer_future_;
    rclcpp::TimerBase::SharedPtr timer_cb_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future_setbool_;
    void callback_timer_future();

    void callback_timer_cb();
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
    timer_future_ = create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&ServiceClientExample::callback_timer_future, this));
    timer_cb_ = create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&ServiceClientExample::callback_timer_cb, this));

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO_STREAM(get_logger(), "initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* timer_future() //{ */

  // a timer callback should never block for too long, so all checks should return after a while
  void ServiceClientExample::callback_timer_future()
  {
    // if the future is already waiting for a result, check if we got one
    if (future_setbool_.valid())
    {
        const std::future_status status = future_setbool_.wait_for(std::chrono::milliseconds(100));;
        if (status == std::future_status::ready)
        {
          // future already got a result, check it and report to user
          const auto result = future_setbool_.get();
          // reset the future
          future_setbool_ = {};
          if (result->success)
          {
            RCLCPP_INFO_STREAM(get_logger(), "future finished, service call successful with message: " << result->message);
          }
          else
          {
            RCLCPP_INFO_STREAM(get_logger(), "future finished, service call reports failure: " << result->message);
          }
        }
        else
        {
          // future isn't ready yet
          RCLCPP_INFO_STREAM(get_logger(), "service future not ready yet, waiting again...");
        }
    }
    // if the future is not waiting for a result, make a new service call
    else
    {
      // you should not send a service request unless the service is ready, otherwise the future will never be ready
      if (service_client_->service_is_ready())
      {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        future_setbool_ = service_client_->async_send_request(request).future.share();
        RCLCPP_INFO_STREAM(get_logger(), "called service using future");
      }
      else
      {
        RCLCPP_WARN_STREAM(get_logger(), "not calling service using future, service not ready!");
      }
    }
  }

  //}

  // | ------------------------ routines ------------------------ |

  // The callback function for service call response.
  // This function will be executed once the other side decides to respond to the service call.
  void callback_future_setbool(const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture fut)
  {
    // the future must be ready now that the callback was called
    const auto result = fut.get();
    if (result->success)
    {
      std::cout << "callback called, service call successful with message: " << result->message << std::endl;
      /* RCLCPP_INFO_STREAM(get_logger(), "[" << get_name() << "]: callback called, service call successful with message: " << result->message); */
    }
    else
    {
      std::cout << "callback called, service call reports failure: " << result->message << std::endl;
      /* RCLCPP_INFO_STREAM(get_logger(), "[" << get_name() << "]: callback called, service call reports failure: " << result->message); */
    }
  }

  /* timer_callback() //{ */

  void ServiceClientExample::callback_timer_cb()
  {
    // it's good to first check if the service server is even ready to be called
    if (service_client_->service_is_ready())
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      future_setbool_ = service_client_->async_send_request(request, &callback_future_setbool).future;
      RCLCPP_INFO_STREAM(get_logger(), "[" << get_name() << "]: called service using callback");
    }
    else
    {
      RCLCPP_INFO_STREAM(get_logger(), "[" << get_name() << "]: not calling service using callback, service not ready!");
    }
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceClientExample)
