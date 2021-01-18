#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>

using namespace std::chrono_literals;

namespace ros2_uav_example
{

/* class ServiceClientExample //{ */

class ServiceClientExample : public rclcpp::Node {
public:
  ServiceClientExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | --------------------- service clients -------------------- |

  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service_client_;

  void callService(void);

  // | ------------------------- timers ------------------------- |

  rclcpp::TimerBase::SharedPtr timer_main_;

  void timerMain();
};

//}

/* ServiceClientExample() //{ */

ServiceClientExample::ServiceClientExample(rclcpp::NodeOptions options) : Node("service_client_example", options) {

  RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: initializing");

  // | --------------------- service client --------------------- |

  service_client_ = this->create_client<std_srvs::srv::SetBool>("~/set_bool_out");

  // | -------------------------- timer ------------------------- |

  timer_main_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / 10.0), std::bind(&ServiceClientExample::timerMain, this));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* timerMain() //{ */

void ServiceClientExample::timerMain(void) {

  if (!is_initialized_) {
    return;
  }

  callService();
}

//}

// | ------------------------ routines ------------------------ |

/* callService() //{ */

void ServiceClientExample::callService(void) {

  RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: calling service");

  auto request  = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;

  {
    // TODO THIS IS HOW YOU ARE SUPPOSED TO WAIT FOR THE SERVICE TO BECOME READY
    // TODO IT MESSES UP SOMETHING WITH THE GRANULARITY OF PUBLISHING
    // TODO THIS IS FROM AN EXAMPLE

    /* while (!service_client_->wait_for_service(1s)) { */
    /*   if (!rclcpp::ok()) { */
    /*     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting."); */
    /*   } */
    /*   RCLCPP_INFO(this->get_logger(), "service not available, waiting again..."); */
    /*   break; */
    /* } */
  }

  // define a callback for the service response
  using ServiceResponseFuture = rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto result = future.get();
    RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: service result received");
  };

  // asynchronous call
  auto result = service_client_->async_send_request(request, response_received_callback);  // with a callback
  /* auto result = service_client_->async_send_request(request); // without a callback */

  {
      // TODO ONE WAY OF WAITING FOR THE RESULT IS TO CHECK THE std::future
      // TODO THIS BLOCKS THE WHOLE NODE FROM GETTING CALLBACKS, THEREFORE, NOTHING WORKS

      /* std::future_status status; */
      /* do { */
      /*   status = result.wait_for(std::chrono::seconds(1)); */
      /*   if (status == std::future_status::deferred) { */
      /*     RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: deferred"); */
      /*   } else if (status == std::future_status::timeout) { */
      /*     RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: timeout"); */
      /*   } else if (status == std::future_status::ready) { */
      /*     RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: ready"); */
      /*   } else { */
      /*     RCLCPP_INFO(this->get_logger(), "[ServiceClientExample]: something else"); */
      /*   } */
      /* } while (status != std::future_status::ready); */
  }

  {
    // TODO ANOTHER WAY HOT TO WAIT FOR THE RESULT IS TO "spin_until_future_complete"

    // TODO THIS IS HOW YOU WOULD WAIT FOR THE RESPONSE IN A NORMAL NODE
    // TODO BUT WE DON'T HAVE THE "NODE"

    /* rclcpp::spin_until_future_complete(node, result); */

    // TODO THIS IS how we can get the "node" from our component, but it crashes
    // TODO BECAUSE WE ARE "ADDING THE COMPONENT AGAIN TO AN EXECUTOR"

    /* rclcpp::spin_until_future_complete(this->get_node_base_interface(), result); */
  }
}

//}

}  // namespace ros2_uav_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_uav_example::ServiceClientExample)
