from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
        robot_localization_dir = get_package_share_directory('robot_localization')
        parameters_file_dir = os.path.join(robot_localization_dir, 'params')
        parameters_file_path = os.path.join(parameters_file_dir, 'merged_ekf_navsat.yaml')
        os.environ['FILE_PATH'] = str(parameters_file_dir)

        final_position = DeclareLaunchArgument(
                'output_final_position',
                default_value='false'),
        output_location = DeclareLaunchArgument(
                'output_location',
                default_value='~/merged_ekf_navsat_debug.txt'),

        ekf_node_odom = Node(
                package='robot_localization', 
                executable='ekf_node', 
                name='ekf_filter_node_odom',
                        output='screen',
                parameters=[parameters_file_path],
                remappings=[('/imu/data', '/marinero/imu_sensor'),
                        ('/odometry/filtered', '/odometry/ekf/local')]           
                )

        ekf_node_map = Node(
                package='robot_localization', 
                executable='ekf_node', 
                name='ekf_filter_node_map',
                        output='screen',
                parameters=[parameters_file_path],
                remappings=[('/imu/data', '/marinero/imu_sensor'),
                        ('/odometry/filtered', '/odometry/ekf/global')]
                )

        navsat_transform_node = Node(
                package='robot_localization', 
                executable='navsat_transform_node', 
                name='navsat_transform',
                        output='screen',
                parameters=[parameters_file_path],
                remappings=[('/imu/data', '/marinero/imu_sensor'),
                        ('/gps/fix', '/marinero/gps_sensor'), 
                        ('/gps/filtered', '/gps/filtered'),
                        # ('/odometry/gps', '/odometry/gps'),
                        ('/odometry/filtered', '/odometry/global')]           
                )

        return LaunchDescription([
                ekf_node_odom,
                ekf_node_map,
                navsat_transform_node
        ])
