#!/usr/bin/env python3

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_name='marinero_simulations'
    dir_name='robot_description'
    xacro_file='marinero_robot.urdf.xacro'
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_4wis4wid = LaunchConfiguration('use_4wis4wid')

    # xacro_location = os.path.join(get_package_share_directory(pkg_name), dir_name, xacro_file)  ## NE RADI U GAZEBO
    # robot_description_xacro = xacro.process_file(xacro_location).toxml()                        ## NE RADI U GAZEBO
    xacro_location = os.path.join(get_package_share_directory(pkg_name))
    robot_description_xacro = os.path.join(xacro_location, dir_name, xacro_file)

    robot_description_config = Command([
        'xacro ', robot_description_xacro, 
        ' use_ros2_control:=', use_ros2_control, 
        ' use_4wis4wid:=', use_4wis4wid, 
        ' sim_mode:=', use_sim_time
    ])

    params = {
        'robot_description': robot_description_config, 
        'use_ros2_control': use_ros2_control, 
        'use_4wis4wid': use_4wis4wid, 
        'use_sim_time': use_sim_time
    }

    robot_state_publisher =Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true'
    )
    
    use_4wis4wid_arg = DeclareLaunchArgument(
        'use_4wis4wid',
        default_value='true',
        description='Choose between Skid Steer or 4WIS4WID controller.'
    )
    
    ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='true',
        description='Choose between gazebo control or ros2 control.'
    )
    return LaunchDescription([
        sim_time_arg,
        ros2_control_arg,
        use_4wis4wid_arg,
        robot_state_publisher
    ]) 