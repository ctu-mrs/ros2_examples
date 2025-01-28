import launch
import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    IfElseSubstitution,
    PythonExpression,
    PathJoinSubstitution,
    EnvironmentVariable,
)

from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "example_plugin_manager"
    pkg_share_path = get_package_share_directory(pkg_name)

    # param loaded from env variable
    uav_type=os.getenv('UAV_TYPE', "")

    ld.add_action(ComposableNodeContainer(

        namespace="nmspc1",
        name='nmspc1_example_plugin_manager',
        package='rclcpp_components',
        executable='component_container_mt',

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='example_plugin_manager::ExamplePluginManager',
                namespace="nmspc1",
                name='example_plugin_manager',

                parameters=[
                        pkg_share_path + '/config/example_plugin_manager.yaml',
                        pkg_share_path + '/config/plugins.yaml',
                        {'uav_type': uav_type},
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