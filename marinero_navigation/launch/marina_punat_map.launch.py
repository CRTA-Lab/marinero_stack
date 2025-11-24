#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, RegisterEventHandler

def generate_launch_description():

    yaml_file = os.path.join(get_package_share_directory('marinero_navigation'),'config/marina_punat_map','marina_punat.yaml')

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': yaml_file}],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': ['map_server']
        }]
    )

    # configure_map_server = ExecuteProcess(
    #     cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'],
    #     output='screen'
    # )

    # activate_map_server = ExecuteProcess(
    #     cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
    #     output='screen'
    # )

    shutdown_map_server = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'shutdown'],
        output='screen'
    )

    on_shutdown_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=map_server_node,
            on_exit=[shutdown_map_server]
        )
    )

    return LaunchDescription([
        map_server_node,
        lifecycle_manager,
        on_shutdown_handler
    ])