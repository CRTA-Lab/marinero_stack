from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    forward_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller']
    )

    forward_velocity_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller']
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller']
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    return LaunchDescription([
        joint_state_broadcaster,
        forward_position_controller,
        forward_velocity_controller,
        # joint_trajectory_controller,
    ])