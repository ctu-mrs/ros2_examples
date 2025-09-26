#!/usr/bin/env python3

import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "ros2_examples"
    pkg_share_path = get_package_share_directory(pkg_name)

    namespace='nmspc1'

    ld.add_action(ComposableNodeContainer(

        namespace='',
        name=namespace+'_tf2_listener_example',
        package='rclcpp_components',

        executable='component_container_mt',
        # executable='component_container',

        composable_node_descriptions=[

            ComposableNode(
                package=pkg_name,
                plugin='ros2_examples::Tf2ListenerExample',
                namespace=namespace,
                name='tf2_listener_example',
            ),

        ],

        output='screen',

    ))

    return ld
