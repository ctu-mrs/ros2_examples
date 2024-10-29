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

    class SmartCounter
    {
      private:
        uint64_t count = 0;
        double rate = 0;
        std::chrono::time_point<std::chrono::steady_clock> init_time = std::chrono::steady_clock::now();

      public:
        void increment()
        {
          if (count == 0)
          {
            init_time = std::chrono::steady_clock::now();
          }
          ++count;
        }

        double get_rate()
        {
          return count / std::chrono::duration<double>(std::chrono::steady_clock::now() - init_time).count();
        }
    };

    /* callback groups assign the execution of callbacks in different threads of the executor */
    /* use a custom callback group for the timer to prevent deadlock with the service clients */
    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    // | --------------------- service clients -------------------- |

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;

    // | ------------------------- timers ------------------------- |

    rclcpp::TimerBase::SharedPtr timer_async_;
    void callback_timer_async();
    SmartCounter counter_async_;

    rclcpp::TimerBase::SharedPtr timer_sync_;
    void callback_timer_sync();
    SmartCounter counter_sync_;
  };

  //}

  /* ServiceClientExample() //{ */

  ServiceClientExample::ServiceClientExample(const rclcpp::NodeOptions & options) : Node("service_client_example", options)
  {
    RCLCPP_INFO_STREAM(get_logger(), "initializing");

    timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // | --------------------- service client --------------------- |

    service_client_ = create_client<std_srvs::srv::SetBool>("~/out_set_bool");

    // | -------------------------- timer ------------------------- |

    // There are two ways you can check the result of the service call:
    // 1. using std::future<Result>
    // 2. using a callback
    // Examples for each case follow.

    /* both the timers will run in one thread as they are part of the the same callback group */
    timer_async_ = create_wall_timer(std::chrono::duration<double>(1.0 / 200.0), std::bind(&ServiceClientExample::callback_timer_async, this), timer_callback_group_);
    timer_sync_ = create_wall_timer(std::chrono::duration<double>(1.0 / 200.0), std::bind(&ServiceClientExample::callback_timer_sync, this), timer_callback_group_);

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO_STREAM(get_logger(), "initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* callback_timer_sync() //{ */

  // a timer callback should never block for too long, so all checks should return after a while
  void ServiceClientExample::callback_timer_sync()
  {
    /* always check if the service is ready before calling */
    if (service_client_->service_is_ready())
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;

      /* the future done callback is being run in a separate thread under the default callback group of the node */
      const auto future_setbool = service_client_->async_send_request(request).future.share();
      RCLCPP_INFO_STREAM(get_logger(), "Service called in sync mode. Timer will wait for the response.");

      /* it is a good practice to check if the future object is not already invalid after the call */
      /* if valid() is false, the future has UNDEFINED behavior */
      if (future_setbool.valid() == false)
      {
        RCLCPP_WARN_STREAM(get_logger(), "Future object was in undefined state.");
        return;
      }

      /* sync call can be done by simply calling the get() which calls the blocking wait() method of the future */
      const auto result = future_setbool.get();
      if (result->success)
      {
        RCLCPP_INFO_STREAM(get_logger(), "Service call successful with message: " << result->message);
      }
      else
      {
        RCLCPP_INFO_STREAM(get_logger(), "Service call reports failure: " << result->message);
      }
    }
    else
    {
      RCLCPP_WARN_STREAM(get_logger(), "Can not call service in sync mode. Service not ready!");
    }
    counter_sync_.increment();
    RCLCPP_INFO_STREAM(get_logger(), "Sync timer real-time rate: " << counter_sync_.get_rate());
  }

  //}

  /* callback_timer_async() //{ */

  void ServiceClientExample::callback_timer_async()
  {
    // it's good to first check if the service server is even ready to be called
    if (service_client_->service_is_ready())
    {
      auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
      request->data = true;
      service_client_->async_send_request(request, [&](const rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture fut)
        /* the callback function for service call response. */
        /* this function will be executed once the other side decides to respond to the service call. */
        /* this function will run in a separate thread under the default callback group of the node */
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
    counter_async_.increment();
    RCLCPP_INFO_STREAM(get_logger(), "Async timer real-time rate: " << counter_async_.get_rate());
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ServiceClientExample)
