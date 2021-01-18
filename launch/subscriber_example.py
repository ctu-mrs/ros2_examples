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
        name=namespace+'_subscriber_example',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_uav_example::SubscriberExample',
                namespace=namespace,
                name='subscriber_example',
                parameters=[
                    pkg_share_path + '/config/subscriber_example.yaml',
                ],
                remappings=[
                    # topics
                    ("~/topic_in", "/nmspc1/publisher_example/topic"),
                ],
            )
        ],
        output='screen',
    ))

    return ld
