#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch Gazebo with hexapod
        ExecuteProcess(
            cmd=['ros2', 'launch', 'hexapod_bringup', 'gazebo.launch.py'],
            output='screen'
        ),
        
        # Launch joint controller
        Node(
            package='hexapod_control',
            executable='joint_controller',
            name='hexapod_joint_controller',
            output='screen'
        ),
        
        # Launch robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen'
        ),
    ])
