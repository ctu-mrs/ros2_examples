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

    custom_config = LaunchConfiguration('custom_config')

    # this adds the args to the list of args available for this launch files
    # these args can be listed at runtime using -s flag
    # default_value is required to if the arg is supposed to be optional at launch time
    ld.add_action(DeclareLaunchArgument(
        'custom_config',
        default_value="",
        description="Path to the custom configuration file. The path can be absolute, starting with '/' or relative to the current working directory",
    ))

    # behaviour:
    #     custom_config == "" => custom_config: ""
    #     custom_config == "/<path>" => custom_config: "/<path>"
    #     custom_config == "<path>" => custom_config: "$(pwd)/<path>"
    custom_config = IfElseSubstitution(
            condition=PythonExpression(['"', custom_config, '" != "" and ', 'not "', custom_config, '".startswith("/")']),
            if_value=PathJoinSubstitution([EnvironmentVariable('PWD'), custom_config]),
            else_value=custom_config
    )

    pkg_name = 'ros2_examples'
    pkg_share_path = get_package_share_directory(pkg_name)

    # param loaded from env variable
    env_var=os.getenv('ENV_VAR', "x500")

    ld.add_action(ComposableNodeContainer(

        namespace='nmspc1',
        name='nmspc1_params_example',
        package='rclcpp_components',
        executable='component_container_mt',

        composable_node_descriptions=[

            ComposableNode(

                package=pkg_name,
                plugin='ros2_examples::ParamsExample',
                namespace='nmspc1',
                name='params_node_ns',

                parameters=[
                        pkg_share_path + '/config/params_example.yaml',
                        {'custom_config': custom_config,
                        'env_var': env_var},
                    ],

                # remappings=[
                #     # topics
                #     ('~/topic_out', '~/topic"),
                # ],

                ),

            ],

        output='screen',
        ))

    return ld
