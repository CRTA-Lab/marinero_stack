#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name="marinero_simulations"
    dir_name="robot_description"
    xacro_file="rsp_marinero_robot.urdf.xacro"

    xacro_location = os.path.join(get_package_share_directory(pkg_name), dir_name, xacro_file)
    robot_description_xacro = xacro.process_file(xacro_location).toxml()

    rviz2_base = os.path.join(get_package_share_directory(pkg_name), "config")
    rviz2_full_config = os.path.join(rviz2_base, 'rsp_marinero_rviz.rviz')

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_xacro}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    rviz2_node = Node(
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=['-d', rviz2_full_config],
        )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])