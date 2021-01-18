import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "ros2_examples"
    pkg_share_path = get_package_share_directory(pkg_name)

    namespace='nmspc1'
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_service_server_example',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_examples::ServiceServerExample',
                namespace=namespace,
                name='service_server_example',
                parameters=[
                    pkg_share_path + '/config/service_server_example.yaml',
                ],
                remappings=[
                    # services
                    ("~/set_bool_in", "~/set_bool"),
                ],
            )
        ],
        output='screen',
    ))

    return ld
