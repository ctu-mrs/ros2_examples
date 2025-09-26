import launch
from ament_index_python import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'vision_examples'
    pkg_share_path = get_package_share_directory(pkg_name)
    namespace='nmspc1'

    ld.add_action(ComposableNodeContainer(

        namespace = namespace,
        name = 'component_publisher_example',
        package='rclcpp_components',

        executable='component_container_mt',

        composable_node_descriptions=[

            # possible to add multiple nodes to this container
            ComposableNode(
                package=pkg_name,
                plugin='vision_examples::ExampleImageTransport',
                namespace=namespace,
                name='example_image_transport',

                # parameters=[
                #     pkg_share_path + '/config/publisher_example.yaml',
                #     ],

                remappings=[
                    # topics
                    ("~/image_in", "/dummy_image"),
                    ],
                )
            ],

        output='screen',

        ))

    return ld