import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument, 
                            TimerAction, RegisterEventHandler, LogInfo, EmitEvent)
from launch.event_handlers import OnProcessExit


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_4wis4wid = LaunchConfiguration('use_4wis4wid')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw_pose = LaunchConfiguration('yaw_pose')

    pkg_name='marinero_simulations'

    rviz2_base = os.path.join(get_package_share_directory(pkg_name), 'config')
    segmented_rviz2_config = os.path.join(rviz2_base, 'seg_marinero_rviz.rviz')
    marina_rviz2_config = os.path.join(rviz2_base, 'marina_marinero_rviz.rviz')
    gazebo_params_file = os.path.join(get_package_share_directory(pkg_name),'config','gazebo_params.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory(pkg_name),'worlds','marina_base.world'),
        # default_value=os.path.join(get_package_share_directory(pkg_name),'worlds','marina_world.world'),    # marina world when no segmentation is wanted
        description='Full path to new world.'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use sim time if true.'
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

    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value= # '0.68', # zone A
                        # '194.195', # zone A
                        # '189.384', # zone A
                        # '212.37', # zone B
                        # '199.80', # zone B
                        # '191.31', # zone C
                        # '-45.826', # zone C
                        '197.4', # docking station
        description='Define x coordinate when spawning marinero robot'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value= # '0.70', # zone A
                        # '50.486', # zone A
                        # '236.609', # zone A
                        # '388.67', # zone B
                        # '651.51', # zone B
                        # '826.93', # zone C
                        # '711.306', # zone C
                        '280.835', # docking station
        description='Define y coordinate when spawning marinero robot'
    )

    direction_arg = DeclareLaunchArgument(
        'yaw_pose',
        default_value= # '0.85', # zone A
                        # '-3.025', # zone A
                        # '2.481', # zone A
                        # '2.51', # zone B
                        # '2.288', # zone B
                        # '-2.332', # zone C
                        # '0.856', # zone C
                        '-3.0755', # docking station
        description='Direction in which the robot will be oriented'
    )

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')
            ]
        ),
        launch_arguments={
            'world': world,
            'extra_gazebo_args': f'--ros-args --params_file{gazebo_params_file}',
        }.items(),
    )

    launch_robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory(pkg_name),'launch','robot_state_publisher.launch.py')
            ]
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_ros2_control': use_ros2_control,
            'use_4wis4wid': use_4wis4wid
            }.items()
    )

    launch_controller_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory(pkg_name),'launch','controllers.launch.py')
            ]
        ),
        condition=IfCondition(use_ros2_control)
    )

    map_odom_trans_publisher_01 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments="--x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
        condition=IfCondition(use_ros2_control)
    )

    map_odom_trans_publisher_02 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments="--x 0 --y 0 --z -1.26 --roll 0 --pitch 0 --yaw 0 --frame-id map --child-frame-id odom".split(' '),
        condition=UnlessCondition(use_ros2_control)
    )

    marinero_spawner_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "marinero",
            "-x", x_pose,
            "-y", y_pose,
            "-z", "1.30",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", yaw_pose
        ],
        output= "screen"
    )

    _4wis4wid_drive_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg_name),'launch','_4wis4wid_drive_joystick.launch.py')
        ]),
        condition=IfCondition(use_ros2_control)
    )

    skid_steer_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(pkg_name),'launch','skid_steer_joystick.launch.py')
        ]),
        condition=UnlessCondition(use_ros2_control)
    )

    marina_marker_node = Node(
        package='marinero_simulations',
        executable='segmented_sdf2marker.py',
        # executable='marina_sdf2marker.py',
    )

    pointcloud_node = Node(
        package='marinero_pointclouds',
        executable='remapped_segmented_pcd_publisher_thread',
        # executable='remapped_marina_pcd_publisher',
    )

    marinero_yolo_node = Node(
        package='marinero_control',
        executable='marinero_yolo',
    )

    rviz_marker_node = Node(
        package='marinero_control',
        executable='marinero_tracker',
    )

    zones_spawner_node = Node(
        package='marinero_simulations',
        executable='segmented_gazebo_publisher.py',
        # executable='marina_gazebo_publisher.py',
        arguments= [x_pose, y_pose],
        output= "screen"
    )

    gazebo_marker_node = Node(
        package='marinero_control',
        executable='gazebo_marker',
    )

    rviz2_node = Node(
        executable='rviz2',
        output='log',
        arguments=['-d', segmented_rviz2_config],
        # arguments=['-d', marina_rviz2_config],
    )

    delayed_gazebo_spawner_node = TimerAction(
        period = 2.0,
        actions = [marinero_spawner_node]
    )

    delayed_controller_manager = TimerAction(
        period = 4.0,
        actions = [launch_controller_manager],
        condition=IfCondition(use_ros2_control)
    )

    marker_nodes = RegisterEventHandler(
        OnProcessExit(
            target_action=marinero_spawner_node,
            on_exit=[rviz_marker_node,
                    gazebo_marker_node,
            ]
        )
    )

    delayed_nodes = TimerAction(
        period = 8.0,
        actions = [marina_marker_node, 
                    pointcloud_node, 
                    marinero_yolo_node,
                ]
    )

    return LaunchDescription([
        world_arg,
        sim_time_arg,
        use_4wis4wid_arg,
        ros2_control_arg,
        x_pose_arg,
        y_pose_arg,
        direction_arg,
        launch_gazebo,
        zones_spawner_node,
        launch_robot_state_publisher,
        delayed_gazebo_spawner_node,
        map_odom_trans_publisher_01,
        map_odom_trans_publisher_02,
        _4wis4wid_drive_joy_launch,
        skid_steer_joy_launch,
        delayed_controller_manager,
        delayed_nodes,
        marker_nodes,
        rviz2_node
    ])