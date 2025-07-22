#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    
    # Hexapod walker node (delayed to ensure controllers are loaded)
    hexapod_walker = TimerAction(
        period=8.0,  # Wait 8 seconds for all controllers to be ready
        actions=[
            Node(
                package='hexapod',
                executable='hexapod_walker.py',
                name='hexapod_walker',
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        hexapod_walker
    ])
