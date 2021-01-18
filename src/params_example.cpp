#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace ros2_uav_example
{

/* class ParamsExample //{ */

class ParamsExample : public rclcpp::Node {
public:
  ParamsExample(rclcpp::NodeOptions options);
  bool is_initialized_ = false;

private:
  // | ----------------------- parameters ----------------------- |

  double      floating_point_number_;
  std::string some_string_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult callbackParameters(std::vector<rclcpp::Parameter> parameters);
};

//}

/* ParamsExample() //{ */

ParamsExample::ParamsExample(rclcpp::NodeOptions options) : Node("params_example", options) {

  RCLCPP_INFO(this->get_logger(), "[ParamsExample]: initializing");

  {
    const std::string param = "param_namespace/floating_number";
    this->declare_parameter(param);
    if (!this->get_parameter(param, floating_point_number_)) {
      RCLCPP_ERROR(this->get_logger(), "[ParamsExample]: could not load param '%s'", param.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "[ParamsExample]: LOADED '%s'", param.c_str());
    }
  }

  {
    const std::string param = "some_string";
    this->declare_parameter(param);
    if (!this->get_parameter(param, some_string_)) {
      RCLCPP_ERROR(this->get_logger(), "[ParamsExample]: could not load param '%s'", param.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "[ParamsExample]: LOADED '%s'", param.c_str());
    }
  }

  // parameter from the launch file
  {
    const std::string param = "uav_type";
    std::string uav_type;
    this->declare_parameter(param);
    if (!this->get_parameter(param, uav_type)) {
      RCLCPP_ERROR(this->get_logger(), "[ParamsExample]: could not load param '%s'", param.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "[ParamsExample]: LOADED '%s'", param.c_str());
    }
  }

  // | ----------------------- parameters ----------------------- |

  param_callback_handle_ = add_on_set_parameters_callback(std::bind(&ParamsExample::callbackParameters, this, std::placeholders::_1));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[ParamsExample]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackParameters() //{ */

rcl_interfaces::msg::SetParametersResult ParamsExample::callbackParameters(std::vector<rclcpp::Parameter> parameters) {

  if (!is_initialized_) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason     = "node is not initialized";
    return result;
  }

  RCLCPP_INFO(this->get_logger(), "[ParamsExample]: params updated");

  for (size_t i = 0; i < parameters.size(); i++) {
    std::stringstream ss;
    ss << "{" << parameters[i].get_name() << ", " << parameters[i].value_to_string() << "}";
    RCLCPP_INFO(this->get_logger(), "[ParamsExample]: got parameter: '%s'", ss.str().c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason     = "don't know";

  return result;
}

//}

// | ------------------------ routines ------------------------ |

}  // namespace ros2_uav_example

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_uav_example::ParamsExample)
