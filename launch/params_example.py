import launch
import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def load_custom_config(name, param_file_list = None):

    if param_file_list == None:
        param_file_list = []

    # custom config for param server
    custom_config=os.getenv(name)

    if custom_config:
        param_file_list = param_file_list + [os.path.abspath(custom_config)]

    return param_file_list

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "ros2_examples"
    pkg_share_path = get_package_share_directory(pkg_name)

    # param loaded from env variable
    uav_type=os.getenv('UAV_TYPE', "")

    param_files = load_custom_config("custom_config")

    ld.add_action(ComposableNodeContainer(

        namespace="nmspc1",
        name='nmspc1_params_example',
        package='rclcpp_components',
        executable='component_container_mt',

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='ros2_examples::ParamsExample',
                namespace="nmspc1",
                name='params_example',

                parameters=[
                        pkg_share_path + '/config/params_example.yaml',
                        {"uav_type": uav_type}
                    ] + param_files

                # remappings=[
                #     # topics
                #     ("~/topic_out", "~/topic"),
                # ],

                ),

            ],

        output='screen',
        ))

    return ld
