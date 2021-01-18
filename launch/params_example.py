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

    UAV_TYPE=os.getenv('UAV_TYPE')

    namespace='nmspc1'
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_params_example',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_examples::ParamsExample',
                namespace=namespace,
                name='params_example',
                parameters=[
                    pkg_share_path + '/config/params_example.yaml',
                    {"uav_type": UAV_TYPE}
                ],
                # remappings=[
                #     # topics
                #     ("~/topic_out", "~/topic"),
                # ],
            ),
        ],
        output='screen',
    ))

    return ld
