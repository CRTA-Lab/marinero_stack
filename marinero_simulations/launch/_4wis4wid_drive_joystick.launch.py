#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    twist_mux_params = os.path.join(get_package_share_directory('marinero_simulations'),'config','twist_mux_4wis4wid.yaml')

    joy_node = Node(
        package='joy',
        executable='joy_node',
    )

    odometry_node = Node(
        package='marinero_control',
        executable='marinero_odometry_new_model',
    )

    teleop_node = Node(
        package='marinero_control',
        executable='marinero_teleop',
    )

    control_node = Node(
        package='marinero_control',
        executable='marinero_control_navigator',
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params]
    )

    return LaunchDescription([
        joy_node,
        odometry_node,
        teleop_node,
        control_node,
        twist_mux_node
    ])