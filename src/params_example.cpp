#include <rclcpp/rclcpp.hpp>
#include <mrs_lib/param_loader.h>
#include <rclcpp/utilities.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

/* class ParamsExample //{ */

class ParamsExample : public rclcpp::Node
{
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

ParamsExample::ParamsExample(rclcpp::NodeOptions options) : Node("params_example", options)
{
  RCLCPP_INFO(get_logger(), "initializing");
  mrs_lib::ParamLoader pl(*this);

  pl.load_param("param_namespace.floating_number", floating_point_number_);
  pl.load_param("some_string", some_string_);
  const auto uav_type = pl.load_param2<std::string>("uav_type");

  if (!pl.loaded_successfully())
  {
    RCLCPP_ERROR(get_logger(), "Some compulsory parameters were not loaded! Ending node.");
    rclcpp::shutdown(nullptr, "Some compulsory parameters were not loaded");
    return;
  }

  // | ----------------------- parameters ----------------------- |

  param_callback_handle_ = add_on_set_parameters_callback(std::bind(&ParamsExample::callbackParameters, this, std::placeholders::_1));

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;
  RCLCPP_INFO(get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackParameters() //{ */

rcl_interfaces::msg::SetParametersResult ParamsExample::callbackParameters(std::vector<rclcpp::Parameter> parameters)
{
  if (!is_initialized_) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason     = "node is not initialized";
    return result;
  }

  RCLCPP_INFO(get_logger(), "params updated");

  for (size_t i = 0; i < parameters.size(); i++) {
    std::stringstream ss;
    ss << "{" << parameters[i].get_name() << ", " << parameters[i].value_to_string() << "}";
    RCLCPP_INFO(get_logger(), "got parameter: '%s'", ss.str().c_str());
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason     = "don't know";

  return result;
}

//}

// | ------------------------ routines ------------------------ |

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ParamsExample)
