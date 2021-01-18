import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "ros2_uav_example"
    pkg_share_path = get_package_share_directory(pkg_name)

    namespace='nmspc1'
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_service_client_example',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_uav_example::ServiceClientExample',
                namespace=namespace,
                name='service_client_example',
                parameters=[
                    pkg_share_path + '/config/service_client_example.yaml',
                ],
                remappings=[
                    # services
                    ("~/set_bool_out", "/nmspc1/service_server_example/set_bool"),
                ],
            )
        ],
        output='screen',
    ))

    return ld
