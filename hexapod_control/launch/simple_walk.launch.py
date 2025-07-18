#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch simple walking controller
        Node(
            package='hexapod_control',
            executable='walking_controller',
            name='simple_walking_controller',
            output='screen'
        ),
        
        # Launch robot state publisher (you may need to provide URDF parameter)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),
    ])
