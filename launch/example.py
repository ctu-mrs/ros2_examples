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

    UAV_TYPE=os.getenv('UAV_TYPE')

    namespace='uav1'
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_ros2_uav_example',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_uav_example::Example',
                namespace=namespace,
                name='ros2_uav_example',
                parameters=[
                    pkg_share_path + '/config/example1.yaml',
                    {"uav_type": UAV_TYPE}
                ],
                remappings=[
                    # topics
                    ("~/topic_out", "~/topic"),
                    ("~/topic_in", "/uav2/ros2_uav_example/topic"),
                    # services
                    ("~/set_bool_in", "~/set_bool"),
                    ("~/set_bool_out", "/uav2/ros2_uav_example/set_bool"),
                ],
            ),
        ],
        output='screen',
    ))

    namespace='uav2'
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_ros2_uav_example',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_uav_example::Example',
                namespace=namespace,
                name='ros2_uav_example',
                parameters=[
                    pkg_share_path + '/config/example2.yaml',
                    {"uav_type": UAV_TYPE}
                ],
                remappings=[
                    # topics
                    ("~/topic_out", "~/topic"),
                    ("~/topic_in", "/uav1/ros2_uav_example/topic"),
                    # services
                    ("~/set_bool_in", "~/set_bool"),
                    ("~/set_bool_out", "/uav1/ros2_uav_example/set_bool"),
                ],
            )
        ],
        output='screen',
    ))

    return ld
