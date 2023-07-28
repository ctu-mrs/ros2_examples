#include <rclcpp/rclcpp.hpp>

namespace utils
{
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
}
