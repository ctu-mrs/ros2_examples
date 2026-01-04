import os
from pathlib import Path
import launch
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

class LaunchHelper:

    def __init__(self, pkg_name, node_name, node_type, namespace=None, thread_count=2):
        self.ld = launch.LaunchDescription()

        self.pkg_name = pkg_name

        self.pkg_share_path = get_package_share_directory(pkg_name)
        self.namespace = namespace if namespace is not None else pkg_name
        self.node_name = node_name
        self.plugin_type = node_type
        self.thread_count = thread_count
        self.parameters = list()
        self.remaps = list()

        print(f'\nCreating a launch file')
        print(f'Package name: [{self.pkg_name}] and namespace: [{self.namespace}]')
        print(f'Node name: [{self.node_name}] of type: [{self.plugin_type}]')

        
    
    def add_launch_arg(self, name, default, *, description='Default description'):
        arg = LaunchConfiguration(name)

        # this adds the args to the list of args available for this launch files
        # these args can be listed at runtime using -s flag
        # default_value is required to if the arg is supposed to be optional at launch time
        self.ld.add_action(DeclareLaunchArgument(
            name,
            default_value=default,
            description=description,
        ))

        self.parameters.append({name: arg})
        print(f'Added launch arguement [{name}]')

    def add_custom_config(self, *, name='custom_config', default='Default value', description='Default description'):
        custom_config = LaunchConfiguration(name)

        # this adds the args to the list of args available for this launch files
        # these args can be listed at runtime using -s flag
        # default_value is required to if the arg is supposed to be optional at launch time
        self.ld.add_action(DeclareLaunchArgument(
            name,
            default_value=default,
            description=description,
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

        self.parameters.append({name: custom_config})
        print(f'Added arguement [custom_config]')

    def add_param_file(self, yaml_file):
        if not os.path.exists(yaml_file):
            raise ValueError(f'[{yaml_file}] file does not exist')

        self.parameters.append(yaml_file)
        print(f'Added params from file {yaml_file}')
    
    def add_param_files_from_directory(self, directory=None):
        resolved_directory = directory if directory is not None else Path(self.pkg_share_path + '/config')
        if not os.path.exists(resolved_directory):
            raise ValueError(f'[{resolved_directory}] does not exist')

        self.parameters.extend(list(resolved_directory.rglob('*.yaml')))
        print(f'Added all the params files found in the path {resolved_directory}')
    
    def add_remapping(self, *, remap_from, remap_to):
        self.remaps.append((remap_from, remap_to))
        print(f'Added remapping from: {remap_from} to: {remap_to}')

    def print_description_properties(self):
        print('\n')
        print(f'Container name: {self.pkg_name}_container')
        print(f'Using threads: {self.thread_count}')

        print(f'Node name: {self.node_name}')
        print(f'Parameters: {self.parameters}')
        print(f'Remappings: {self.remaps}')
        print('\n')

    def get_launch_description(self):

        self.ld.add_action(ComposableNodeContainer(
            namespace=self.namespace,
            name=self.pkg_name + '_container',
            package='rclcpp_components',
            executable='component_container_mt',
            output="screen",
            parameters=[
                {'thread_num': self.thread_count},
            ],

            composable_node_descriptions=[

                ComposableNode(

                    package=self.pkg_name,
                    plugin=self.plugin_type,
                    namespace=self.namespace,
                    name=self.node_name,
                    parameters=self.parameters,
                    remappings=self.remaps,
                )
            ],
        ))

        self.print_description_properties()

        return self.ld