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
    rcl_interfaces::msg::SetParametersResult callback_parameters(std::vector<rclcpp::Parameter> parameters);
  };

  //}


  // a helper parameter loading function
  template <class T>
  bool parse_param(const std::string &param_name, T &param_dest, rclcpp::Node& node)
  {
    // each parameter has to firstly be declared to ROS2 parameter server
    node.declare_parameter<T>(param_name); // for Galactic and newer
    // then we can attempt to load its value from the server
    if (!node.get_parameter(param_name, param_dest))
    {
      RCLCPP_ERROR_STREAM(node.get_logger(), "Could not load param '" << param_name << "'");
      return false;
    }
    else
    {
      RCLCPP_INFO_STREAM(node.get_logger(), "Loaded '" << param_name << "' = '" << param_dest << "'");
      return true;
    }
  }


  /* ParamsExample() //{ */

  ParamsExample::ParamsExample(rclcpp::NodeOptions options) : Node("params_example", options)
  {

    RCLCPP_INFO(get_logger(), "[ParamsExample]: initializing");

    bool loaded_successfully = true;

    loaded_successfully &= parse_param("param_namespace.floating_number", floating_point_number_, *this);
    loaded_successfully &= parse_param("some_string", some_string_, *this);
    std::string uav_type;
    loaded_successfully &= parse_param("uav_type", uav_type, *this);

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

  // | ------------------------ routines ------------------------ |

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ParamsExample)
