import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, 
                            TimerAction,IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'marinero_navigation'
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch') 

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
        )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        # default_value=os.path.join(get_package_share_directory(pkg_name),'config','dwb_nav2_marinero_skid_steer_params.yaml'),
        # default_value=os.path.join(get_package_share_directory(pkg_name),'config','dwb_nav2_marinero_4wis4wid_drive_params.yaml'),
        default_value=os.path.join(get_package_share_directory(pkg_name),'config','mppi_nav2_marinero_4wis4wid_drive_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
        )

    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(get_package_share_directory(pkg_name),'config','marina_punat_map','marina_punat.yaml'),
        description='Full path to map yaml file to load'
        )

    autostart_arg = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
        )

    respawn_arg = DeclareLaunchArgument(
        'use_respawn', 
        default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
        )

    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('marinero_navigation'),'launch','marina_punat_map.launch.py')
        ])
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir,'localization_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'map': map_file,
                        'params_file': params_file,
                        'use_respawn': use_respawn}.items())

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'params_file': params_file}.items())

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'map': map_file,
                        'params_file': params_file,
                        'use_respawn': use_respawn}.items())

    robot_localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('robot_localization'),'launch','merged_ekf_navsat.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())    

    mapviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('mapviz'),'launch','mapviz.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items())    

    return LaunchDescription([
        sim_time_arg,
        autostart_arg,
        respawn_arg,
        map_file_arg,
        params_file_arg,
        map_launch,
        navigation_launch,
        # robot_localization_launch,
        mapviz_launch,
    ])