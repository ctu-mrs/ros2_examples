#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
#include <ros2_examples/params.h>

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
};

//}

/* ParamsExample() constructor //{ */

ParamsExample::ParamsExample(const rclcpp::NodeOptions options) : Node("param_example", options) {

  RCLCPP_INFO(get_logger(), "initializing");

  bool loaded_successfully = true;
  // | --------------------- load parameters -------------------- |

  RCLCPP_INFO(get_logger(), "Loading using Node");
  loaded_successfully &= utils::load_param("env_var", env_var, *this);
  loaded_successfully &= utils::load_param("floating_point_number", params.floating_point_number, *this);
  loaded_successfully &= utils::load_param("some_string", params.some_string, *this);
  loaded_successfully &= utils::load_param("vec_double", params.vec_double, *this);
  loaded_successfully &= utils::load_param("vec_str", params.vec_str, *this);
  loaded_successfully &= utils::load_param("namespace1.str", params.namespaced_str, *this);
  std::string custom_config = "";
  loaded_successfully &= utils::load_param("custom_config", custom_config, *this);


  if (!loaded_successfully) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters using Node. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "Loading using Sub-Node");
  auto sub_node = this->create_sub_node("params");
  loaded_successfully &= utils::load_param("env_var", env_var, *sub_node);
  loaded_successfully &= utils::load_param("floating_point_number", params.floating_point_number, *sub_node);
  loaded_successfully &= utils::load_param("some_string", params.some_string, *sub_node);
  loaded_successfully &= utils::load_param("vec_double", params.vec_double, *sub_node);
  loaded_successfully &= utils::load_param("vec_str", params.vec_str, *sub_node);
  loaded_successfully &= utils::load_param("namespace1.str", params.namespaced_str, *sub_node);
  loaded_successfully &= utils::load_param("custom_config", custom_config, *this);

  if (!loaded_successfully) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters using Sub-Node. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  // | --------------------- finish the init -------------------- |

  RCLCPP_INFO(get_logger(), "initialized");
}

//}

}  // namespace ros2_examples

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ros2_examples::ParamsExample)
