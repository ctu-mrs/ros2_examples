#include <rclcpp/rclcpp.hpp>

#include <sstream>
#include <string>
#include <vector>

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

  std::string env_var = "I should have an env variable";
  struct ConfigYAMLParams {
    double floating_point_number = 786.00;
    std::string some_string = "I should be changed";
    std::vector<std::string> vec_str{"I", "must", "be", "changed"};
    std::vector<double> vec_double{7.0, 8.0, 6.0};
    std::string namespaced_str = "I should have a namespaced string";
  };
  ConfigYAMLParams params; 

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // | ------------------------- methods ------------------------ |

  // rcl_interfaces::msg::SetParametersResult callback_parameters(std::vector<rclcpp::Parameter> parameters);
  bool load_parameters(const rclcpp::Node& node_ptr);
};

//}

/* ParamsExample() constructor //{ */

ParamsExample::ParamsExample(const rclcpp::NodeOptions options) : Node("param_example", options) {

  RCLCPP_INFO(get_logger(), "initializing");

  bool loaded_successfully = true;
  auto sub_node = this->create_sub_node("params");

  // | --------------------- load parameters -------------------- |

  loaded_successfully &= load_parameters(*this);
  // loaded_successfully &= load_parameters(sub_node);

  if (!loaded_successfully) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
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

bool ParamsExample::load_parameters(const rclcpp::Node& node_ptr) {

  bool loaded_successfully = true;

  node_ptr->declare_parameter<std::string>("env_var");
  loaded_successfully &= node_ptr->get_parameter("env_var", env_var);
  RCLCPP_INFO_STREAM(get_logger(), "[Param]: env_var: " << env_var);

  node_ptr->declare_parameter<double>("floating_point_number");
  loaded_successfully &= node_ptr->get_parameter("floating_point_number", params.floating_point_number);
  RCLCPP_INFO_STREAM(get_logger(), "[Param]: floating_point_number: " << params.floating_point_number);

  node_ptr->declare_parameter<std::string>("some_string");
  loaded_successfully &= node_ptr->get_parameter("some_string", params.some_string);
  RCLCPP_INFO_STREAM(get_logger(), "[Param]: some_string: " << params.some_string);

  node_ptr->declare_parameter<std::vector<double>>("vec_double");
  loaded_successfully &= node_ptr->get_parameter("vec_double", params.vec_double);

  std::stringstream tmp_str;
  tmp_str << "[";
  for (const auto&  val: params.vec_double) {
    tmp_str << val <<", ";
  }
  tmp_str << "]";

  RCLCPP_INFO_STREAM(get_logger(), "[Param]: vec_double: " << tmp_str.str());

  node_ptr->declare_parameter<std::vector<std::string>>("vec_str");
  loaded_successfully &= node_ptr->get_parameter("vec_str", params.vec_str);
  std::stringstream tmp_double_str;
  tmp_double_str << "[";
  for (const auto&  val: params.vec_double) {
    tmp_double_str << val <<", ";
  }
  tmp_double_str << "]";
  RCLCPP_INFO_STREAM(get_logger(), "[Param]: vec_str: " << tmp_double_str.str());

  node_ptr->declare_parameter<std::string>("namespace1.str");
  loaded_successfully &= node_ptr->get_parameter("namespace1.str", params.namespaced_str);
  RCLCPP_INFO_STREAM(get_logger(), "[Param]: namespace1.str: " << params.namespaced_str);

  return loaded_successfully;
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ParamsExample)
