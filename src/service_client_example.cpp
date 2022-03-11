#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{
  /* class ServiceClientExample //{ */

  class ServiceClientExample : public rclcpp::Node
  {
  public:
    ServiceClientExample(rclcpp::NodeOptions options);

  private:
    // | --------------------- service clients -------------------- |

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;


    // | ------------------------- timers ------------------------- |

    rclcpp::TimerBase::SharedPtr timer_future_;
    rclcpp::TimerBase::SharedPtr timer_callback_;

    rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture setbool_future_;
    void timer_future();

    void setbool_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future);
    void timer_callback();
  };

  //}

  /* ServiceClientExample() //{ */

  ServiceClientExample::ServiceClientExample(rclcpp::NodeOptions options) : Node("service_client_example", options)
  {
    RCLCPP_INFO(get_logger(), "[ServiceClientExample]: initializing");

    // | --------------------- service client --------------------- |

    service_client_ = create_client<std_srvs::srv::SetBool>("~/set_bool_out");

    // | -------------------------- timer ------------------------- |

    // There are two ways you can check the result of the service call:
    // 1. using std::future<Result>
    // 2. using a callback
    // Examples for each case follow.

    timer_future_ = create_wall_timer(std::chrono::duration<double>(1.0 / 2.0), std::bind(&ServiceClientExample::timer_future, this));
    timer_callback_ = create_wall_timer(std::chrono::duration<double>(1.0 / 2.0), std::bind(&ServiceClientExample::timer_callback, this));

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(get_logger(), "[ServiceClientExample]: initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* timer_future() //{ */

  // a timer callback should never block for too long, so all checks should return after a while
  void ServiceClientExample::timer_future()
  {
    // if the future is already waiting for a result, check if we got one
    if (setbool_future_.valid())
    {
        const std::future_status status = setbool_future_.wait_for(std::chrono::seconds(1));;
        if (status == std::future_status::ready)
        {
          // future already got a result, check it and report to user
          const auto result = setbool_future_.get();
          // reset the future
          setbool_future_ = {};
          if (result->success)
            RCLCPP_INFO_STREAM(get_logger(), "[ServiceClientExample]: future finished, service call successful with message: " << result->message);
          else
            RCLCPP_INFO_STREAM(get_logger(), "[ServiceClientExample]: future finished, service call reports failure: " << result->message);
        }
        else
          // future isn't ready yet
          RCLCPP_INFO(get_logger(), "[ServiceClientExample]: service future not ready yet, waiting again...");
    }
    // if the future is not waiting for a result, make a new service call
    else
    {
      // you should not send a service request unless the service is ready, otherwise the future will never be ready
      if (service_client_->service_is_ready())
      {
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = true;
        setbool_future_ = service_client_->async_send_request(request);
        RCLCPP_INFO(get_logger(), "[ServiceClientExample]: called service using future");
      }
      else
        RCLCPP_WARN(get_logger(), "[ServiceClientExample]: not calling service using future, service not ready!");
    }
  }

  //}

  /* timer_callback() //{ */

  void ServiceClientExample::timer_callback()
  {
    // it's good to firstly check if the service server is even ready to be called
    if (service_client_->service_is_ready())
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      setbool_future_ = service_client_->async_send_request(request);
      RCLCPP_INFO(get_logger(), "[ServiceClientExample]: called service using callback");
    }
    else
      RCLCPP_WARN(get_logger(), "[ServiceClientExample]: not calling service using callback, service not ready!");
  }

  //}

  // | ------------------------ routines ------------------------ |

  // The callback function for service call response.
  // This function will be executed once the other side decides to respond to the service call.
  void ServiceClientExample::setbool_callback(rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future)
  {
    // the future must be ready now that the callback was called
    const auto result = setbool_future_.get();
    if (result->success)
      RCLCPP_INFO_STREAM(get_logger(), "[ServiceClientExample]: callback called, service call successful with message: " << result->message);
    else
      RCLCPP_INFO_STREAM(get_logger(), "[ServiceClientExample]: callback called, service call reports failure: " << result->message);
  }

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceClientExample)
