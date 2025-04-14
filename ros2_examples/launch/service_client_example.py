#!/usr/bin/env python3

import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = 'ros2_examples'

    pkg_share_path = get_package_share_directory(pkg_name)

    namespace = pkg_name

    ld.add_action(ComposableNodeContainer(

        namespace = namespace,
        name = 'component_service_client_example',

        package='rclcpp_components',

        executable='component_container_mt',

        composable_node_descriptions=[

            ComposableNode(
                package=pkg_name,
                plugin='ros2_examples::ServiceClientExample',
                namespace=namespace,
                name='service_client_example',

                parameters=[
                    pkg_share_path + '/config/service_client_example.yaml',
                    ],

                remappings=[
                    # services
                    ('~/out_set_bool', '/' + pkg_name + '/service_server_example/set_bool'),
                    ],
                )

            ],

        output='screen',
        ))

    return ld
