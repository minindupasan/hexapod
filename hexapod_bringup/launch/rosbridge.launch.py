#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='9090',
        description='Port for rosbridge server'
    )
    
    address_arg = DeclareLaunchArgument(
        'address',
        default_value='0.0.0.0',
        description='Address for rosbridge server'
    )

    return LaunchDescription([
        port_arg,
        address_arg,
        
        # Rosbridge server for web interface
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'address': LaunchConfiguration('address'),
                'authenticate': False,
                'retry_startup_delay': 5,
                'fragment_timeout': 600,
                'delay_between_messages': 0,
                'max_message_size': None,
                'unregister_timeout': 3600000,
                'use_compression': False,
                'websocket_ping_interval': 0,
                'websocket_ping_timeout': 30,
                'websocket_null_origin': True
            }]
        ),
        
        # ROS API node for web interface
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            output='screen'
        )
    ])
