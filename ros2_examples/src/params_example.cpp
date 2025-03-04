#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/param_loader.h>

using namespace std::chrono_literals;

namespace ros2_examples
{

// using namespace utils;

/* class ParamsExample //{ */

class ParamsExample : public rclcpp::Node {
public:
  ParamsExample(const rclcpp::NodeOptions options);

private:
  // | ----------------------- parameters ----------------------- |

  double      floating_point_number_;
  std::string some_string_;
  std::string uav_type;

  // OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // | ------------------------- methods ------------------------ |

  // rcl_interfaces::msg::SetParametersResult callback_parameters(std::vector<rclcpp::Parameter> parameters);
};

//}

/* ParamsExample() constructor //{ */

ParamsExample::ParamsExample(const rclcpp::NodeOptions options) : Node("params_example", options) {

  RCLCPP_INFO(get_logger(), "initializing");

  // bool loaded_successfully = true;
  auto m_node = this->create_sub_node("params");

  // | --------------------- load parameters -------------------- |

  mrs_lib::ParamLoader param_loader{m_node, std::string{"ParamsExample"}};

  std::string custom_config = "";
  // param_loader.loadParam("param_namespace/floating_number", floating_point_number_);
  param_loader.loadParam("some_string", some_string_);
  param_loader.loadParam("some_string", some_string_);
  // param_loader.loadParam("custom_config", custom_config);

  // if (custom_config == "") {
  //   RCLCPP_WARN(get_logger(), "custom_config is empty");
  // }

  param_loader.loadParam("uav_type", uav_type);

  auto param_list = m_node->list_parameters(std::vector<std::string>(), 0);

  for (const auto& param: param_list.names) {
    std::cout << "params: " << param << std::endl;
  
  }

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
    // rclcpp::shutdown();
    return;
  }

  // | --------------- bind pararm server callback -------------- |

  // param_callback_handle_ = add_on_set_parameters_callback(std::bind(&ParamsExample::callback_parameters, this, std::placeholders::_1));

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callback_parameters() //{ */

// rcl_interfaces::msg::SetParametersResult ParamsExample::callback_parameters(std::vector<rclcpp::Parameter> parameters) {
//   rcl_interfaces::msg::SetParametersResult result;

//   // Note that setting a parameter to a nonsensical value (such as setting the `param_namespace.floating_number` parameter to `hello`)
//   // doesn't have any effect - it doesn't even call this callback.
//   for (auto& param : parameters) {

//     RCLCPP_INFO_STREAM(get_logger(), "got parameter: '" << param.get_name() << "' with value '" << param.value_to_string() << "'");

//     if (param.get_name() == "param_namespace.floating_number") {

//       floating_point_number_ = param.as_double();

//     } else if (param.get_name() == "some_string") {

//       some_string_ = param.as_string();

//     } else {

//       RCLCPP_WARN_STREAM(get_logger(), "parameter: '" << param.get_name() << "' is not dynamically reconfigurable!");
//       result.successful = false;
//       result.reason     = "Parameter '" + param.get_name() + "' is not dynamically reconfigurable!";
//       return result;
//     }
//   }

//   RCLCPP_INFO(get_logger(), "params updated");
//   result.successful = true;
//   result.reason     = "OK";

//   return result;
// }

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ParamsExample)
