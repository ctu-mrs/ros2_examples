#!/usr/bin/env python3

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "sub_pub_torture_test"
    pkg_share_path = get_package_share_directory(pkg_name)
    namespace=''

    ld.add_action(ComposableNodeContainer(

        namespace = namespace,
        name = 'subscribers_server',
        package='rclcpp_components',
        executable='component_container_mt',

        parameters=[
            {'thread_num': 16},
        ],

        composable_node_descriptions=[

            ComposableNode(
                package=pkg_name,
                plugin='sub_pub_torture_test::Subscribers',
                namespace=namespace,
                name='subscribers',

                parameters=[
                    pkg_share_path + '/config/subscribers.yaml',
                    ],
                )

            ],

        output='screen',
        ))

    return ld
