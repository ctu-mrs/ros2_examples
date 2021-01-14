import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "ros2_uav_example"
    pkg_share_path = get_package_share_directory(pkg_name)

    namespace='uav1'
    ld.add_action(ComposableNodeContainer(
        namespace='',
        name=namespace+'_ros2_uav_example',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_uav_example::Example',
                namespace=namespace,
                name='ros2_uav_example',
                parameters=[
                    pkg_share_path + '/config/example1.yaml',
                    {"launch_param": "it works!"}
                ],
                remappings=[
                    ("~/topic_out", "~/topic"),
                    ("~/topic_in", "/uav2/ros2_uav_example/topic"),
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
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package=pkg_name,
                plugin='ros2_uav_example::Example',
                namespace=namespace,
                name='ros2_uav_example',
                parameters=[
                    pkg_share_path + '/config/example2.yaml',
                    {"launch_param": "it works!"}
                ],
                remappings=[
                    ("~/topic_out", "~/topic"),
                    ("~/topic_in", "/uav1/ros2_uav_example/topic"),
                ],
            )
        ],
        output='screen',
    ))

    return ld
