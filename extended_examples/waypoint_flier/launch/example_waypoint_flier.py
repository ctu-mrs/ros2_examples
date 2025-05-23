#!/usr/bin/env python3

import launch
import os
import sys

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
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

    pkg_name = "example_waypoint_flier"

    this_pkg_path = get_package_share_directory(pkg_name)
    namespace='waypoint_flier'

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    declare_standalone = DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start a as a standalone or load into an existing container.'
    )

    ld.add_action(declare_standalone)

    # #} end of standalone

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    )

    ld.add_action(declare_container_name)

    # #} end of container_name

    # #{ env-based params

    uav_name=os.getenv('UAV_NAME', "uav1")
    use_sim_time=os.getenv('USE_SIM_TIME', "false") == "true"

    # #} end of env-based params

    # #{ log_level

    ld.add_action(DeclareLaunchArgument(name='log_level', default_value='info'))

    # #} end of log_level

    # #{ waypoint_flier node

    waypoint_flier_node = ComposableNode(

        package=pkg_name,
        plugin='example_waypoint_flier::ExampleWaypointFlier',
        namespace=uav_name,
        name='waypoint_flier',
        parameters=[
            {"uav_name": uav_name},
            {"topic_prefix": "/" + uav_name},
            {"enable_profiler": False},
            {"use_sim_time": use_sim_time},
            {'config': this_pkg_path + '/config/example_waypoint_flier.yaml'},
        ],

        remappings=[
            # subscribers
            ("~/odom_uav_in", "estimation_manager/odom_main"),
            ("~/control_manager_diagnostics_in", "control_manager/diagnostics"),
            ("~/odom_gt_in", "ground_truth"),
            # publishers
            ("~/reference_out", "control_manager/reference"),
            ("~/dist_to_waypoint_out", "~/dist_to_waypoint"),
            # services in
            ("~/start_waypoints_following_in", "~/start_waypoints_following"),
            ("~/stop_waypoints_following_in", "~/stop_waypoints_following"),
            ("~/fly_to_first_waypoint_in", "~/fly_to_first_waypoint"),
            # services out
            ("~/land_out", "uav_manager/land"),
        ],
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[waypoint_flier_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #} end of waypoint_flier node

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output="screen",
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        # prefix=['debug_roslaunch ' + os.ttyname(sys.stdout.fileno())],
        composable_node_descriptions=[waypoint_flier_node],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #} end of own container

    return ld
