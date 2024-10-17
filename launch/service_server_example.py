import launch
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    pkg_name = 'ros2_examples'
    # path to the ROS pkg to be used to load launch and config files
    pkg_share_path = get_package_share_directory(pkg_name)

    namespace = pkg_name
    # create a container process (single thread executor) that will load and run the node
    container = ComposableNodeContainer(
        namespace = namespace,
        name = 'container_service_server_example',
        # pkg and type of the executor to be used for running the node
        # NOTE: the executable param decides if the node is going to be run by a SingleThreadedExecutor or MultiThreadedExecutor 
        package='rclcpp_components',
        # NOTE: component_container is SingleThreadedExecutor and component_container_mt is MultiThreadedExecutor 
        executable='component_container',
        # describe the node
        composable_node_descriptions=[
            # possible to add multiple nodes to this container
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
                    ('~/set_bool_in', '~/set_bool'),
                ],
            )
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
