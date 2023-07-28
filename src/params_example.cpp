#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace ros2_examples
{

  /* class ParamsExample //{ */

  class ParamsExample : public rclcpp::Node
  {
  public:
    ParamsExample(rclcpp::NodeOptions options);

  private:
    // | ----------------------- parameters ----------------------- |

    double floating_point_number_;
    std::string some_string_;

    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // | ------------------------- methods ------------------------ |
    rcl_interfaces::msg::SetParametersResult callback_parameters(std::vector<rclcpp::Parameter> parameters);
  };

  //}


  /* parse_param() method //{ */
  // a helper parameter loading function
  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
  {
    // firstly, the parameter has to be specified (together with its type), which can throw an exception
    try
    {
      node.declare_parameter<T>(param_name); // for Galactic and newer, the type has to be specified here
    }
    catch (const std::exception& e)
    {
      // this can happen if (see http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Node.html#_CPPv4N6rclcpp4Node17declare_parameterERKNSt6stringERKN6rclcpp14ParameterValueERKN14rcl_interfaces3msg19ParameterDescriptorEb):
      // * parameter has already been declared              (rclcpp::exceptions::ParameterAlreadyDeclaredException)
      // * parameter name is invalid                        (rclcpp::exceptions::InvalidParametersException)
      // * initial value fails to be set                    (rclcpp::exceptions::InvalidParameterValueException, not sure what exactly this means)
      // * type of the default value or override is wrong   (rclcpp::exceptions::InvalidParameterTypeException, the most common one)
      RCLCPP_ERROR_STREAM(node.get_logger(), "Could not load param '" << param_name << "': " << e.what());
      return false;
    }
  
    // then we can attempt to load its value from the server
    if (node.get_parameter(param_name, param_dest))
    {
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
      return true;
    }
    else
    {
      // this branch should never happen since we *just* declared the parameter
      RCLCPP_ERROR_STREAM(node.get_logger(), "Could not load param '" << param_name << "': Not declared!");
      return false;
    }
  }
  
  template <class T>
  T parse_param2(const std::string &param_name, bool& ok_out, rclcpp::Node& node)
  {
    T out;
    ok_out = parse_param(param_name, out, node);
    return out;
  }
  //}

  /* ParamsExample() constructor //{ */

  ParamsExample::ParamsExample(rclcpp::NodeOptions options) : Node("params_example", options)
  {

    RCLCPP_INFO(get_logger(), "[ParamsExample]: initializing");

    bool loaded_successfully = true;

    loaded_successfully &= parse_param("param_namespace.floating_number", floating_point_number_, *this);
    loaded_successfully &= parse_param("some_string", some_string_, *this);
    
    const std::string uav_type = parse_param2<std::string>("uav_type", loaded_successfully, *this);

    if (!loaded_successfully)
    {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    // | ----------------------- parameters ----------------------- |

    param_callback_handle_ = add_on_set_parameters_callback(std::bind(&ParamsExample::callback_parameters, this, std::placeholders::_1));

    // | --------------------- finish the init -------------------- |

    RCLCPP_INFO(get_logger(), "[ParamsExample]: initialized");
  }

  //}

  // | ------------------------ callbacks ----------------------- |

  /* callback_parameters() //{ */

  rcl_interfaces::msg::SetParametersResult ParamsExample::callback_parameters(std::vector<rclcpp::Parameter> parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;

    // Note that setting a parameter to a nonsensical value (such as setting the `param_namespace.floating_number` parameter to `hello`)
    // doesn't have any effect - it doesn't even call this callback.
    for (auto& param : parameters)
    {
      RCLCPP_INFO_STREAM(get_logger(), "[ParamsExample]: got parameter: '" << param.get_name() << "' with value '" << param.value_to_string() << "'");
      if (param.get_name() == "param_namespace.floating_number")
        floating_point_number_ = param.as_double();
      else if (param.get_name() == "some_string")
        some_string_ = param.as_string();
      else
      {
        RCLCPP_WARN_STREAM(get_logger(), "[ParamsExample]: parameter: '" << param.get_name() << "' is not dynamically reconfigurable!");
        result.successful = false;
        result.reason = "Parameter '" + param.get_name() + "' is not dynamically reconfigurable!";
        return result;
      }
    }

    RCLCPP_INFO(get_logger(), "[ParamsExample]: params updated");
    result.successful = true;
    result.reason = "OK";
    return result;
  }

  //}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ParamsExample)
