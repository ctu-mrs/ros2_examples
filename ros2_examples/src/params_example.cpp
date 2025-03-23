#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>
// #include <ros2_examples/params.h>
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

  std::string env_var = "I should have an env variable";
  struct ConfigYAMLParams {
    double floating_point_number = 786.00;
    std::string some_string = "I should be changed";
    std::vector<std::string> vec_str{"I", "must", "be", "changed"};
    std::vector<double> vec_double{7.0, 8.0, 6.0};
  };
  ConfigYAMLParams params; 
};

//}

/* ParamsExample() constructor //{ */

ParamsExample::ParamsExample(const rclcpp::NodeOptions options) : Node("param_example", options) {

  RCLCPP_INFO(get_logger(), "initializing");

  auto param_sub_node = this->create_sub_node("ns1_depth1");
  mrs_lib::ParamLoader param_loader{param_sub_node, std::string{this->get_name()}};

  RCLCPP_INFO_STREAM(get_logger(), "Load the top-level parameters under namespace: " << param_sub_node->get_sub_namespace());
  // top-level params mostly passed as args to the launch file
  param_loader.loadParam("env_var", env_var);
  std::string custom_config = "";
  param_loader.loadParam("custom_config", custom_config);

  // params mostly stored inside the yaml files
  param_loader.loadParam("floating_point_number", params.floating_point_number);
  param_loader.loadParam("some_string", params.some_string);
  param_loader.loadParam("vec_double", params.vec_double);
  param_loader.loadParam("vec_str", params.vec_str);
  
  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters using Node. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  param_sub_node = this->create_sub_node("ns2_depth1");
  param_loader = mrs_lib::ParamLoader{param_sub_node, std::string{this->get_name()}};

  RCLCPP_INFO_STREAM(get_logger(), "Load the top-level parameters under namespace: " << param_sub_node->get_sub_namespace());
  param_loader.loadParam("floating_point_number", params.floating_point_number);
  param_loader.loadParam("some_string", params.some_string);
  param_loader.loadParam("vec_double", params.vec_double);
  param_loader.loadParam("vec_str", params.vec_str);
  
  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters using Node. Shutting down.");
    rclcpp::shutdown();
    return;
  }

  // load lower-level namespaces using sub-nodes of the main param_sub_node
  auto child_sub_node = param_sub_node->create_sub_node("ns1_depth2");
  mrs_lib::ParamLoader param_loader2{child_sub_node, std::string{this->get_name()}};
  ConfigYAMLParams param_tmp;

  // | --------------------- load parameters -------------------- |

  RCLCPP_INFO_STREAM(get_logger(), "Load using sub-node with namespace: " << child_sub_node->get_sub_namespace());
  param_loader2.loadParam("floating_point_number", param_tmp.floating_point_number);
  param_loader2.loadParam("some_string", param_tmp.some_string);
  param_loader2.loadParam("vec_double", param_tmp.vec_double);
  param_loader2.loadParam("vec_str", param_tmp.vec_str);

  ConfigYAMLParams param_tmp2;
  param_loader2.loadParam("ns1_depth3.floating_point_number", param_tmp.floating_point_number);
  param_loader2.loadParam("ns1_depth3.some_string", param_tmp.some_string);
  param_loader2.loadParam("ns1_depth3.vec_double", param_tmp.vec_double);
  param_loader2.loadParam("ns1_depth3.vec_str", param_tmp.vec_str);

  if (!param_loader2.loadedSuccessfully()) {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not load all non-optional parameters using Node. Shutting down.");
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
