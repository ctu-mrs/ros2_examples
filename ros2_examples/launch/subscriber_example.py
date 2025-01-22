import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "ros2_examples"
    pkg_share_path = get_package_share_directory(pkg_name)
    namespace='nmspc1'

    ld.add_action(ComposableNodeContainer(

        namespace = namespace,
        name = 'component_publisher_example',
        package='rclcpp_components',

        executable='component_container_mt',

        composable_node_descriptions=[

            ComposableNode(
                package=pkg_name,
                plugin='ros2_examples::SubscriberExample',
                namespace=namespace,
                name='subscriber_example',
                parameters=[
                    pkg_share_path + '/config/subscriber_example.yaml',
                    ],
                remappings=[
                    # topics
                    ("~/topic_fast_in", "/nmspc1/publisher_example/topic_fast"),
                    ("~/topic_slow_in", "/nmspc1/publisher_example/topic_slow"),
                    ("~/topic_irregular_in", "/nmspc1/publisher_example/topic_irregular"),
                    ],
                )

            ],

        output='screen',
        ))

    return ld
