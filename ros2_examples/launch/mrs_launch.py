#!/usr/bin/env python3
import os
from launch_helper.helper import LaunchHelper

def generate_launch_description():

    launch_desc = LaunchHelper(pkg_name='ros2_examples', node_name='param_node', node_type='ros2_examples::ParamsExample', namespace='nmmspc')

    launch_desc.add_launch_arg('env_var', os.getenv('ENV_VAR', 'dummy'))
    launch_desc.add_custom_config(default='')
    launch_desc.add_param_files_from_directory()
    launch_desc.add_remapping(remap_from='~/dummy_out', remap_to='/dummy_out')

    return launch_desc.get_launch_description()
