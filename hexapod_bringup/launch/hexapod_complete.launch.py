#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # Launch arguments
    enable_bridge_arg = DeclareLaunchArgument(
        'enable_bridge',
        default_value='false',
        description='Enable rosbridge server for web interface'
    )
    
    enable_walking_arg = DeclareLaunchArgument(
        'enable_walking',
        default_value='false',
        description='Start walking controller automatically'
    )
    
    enable_test_arg = DeclareLaunchArgument(
        'enable_test',
        default_value='false',
        description='Start test controller for joint movement verification'
    )
    
    # Include the main Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hexapod_bringup'),
                'launch',
                'gazebo.launch.py'
            ])
        ])
    )
    
    # Rosbridge server for web interface
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0'
        }],
        output='screen'
    )
    
    # Walking controller
    walking_controller_node = Node(
        package='hexapod_control',
        executable='ros2_control_walking_controller.py',
        name='walking_controller',
        condition=IfCondition(LaunchConfiguration('enable_walking')),
        output='screen'
    )
    
    # Test controller
    test_controller_node = Node(
        package='hexapod_control',
        executable='test_controller.py',
        name='test_controller',
        condition=IfCondition(LaunchConfiguration('enable_test')),
        output='screen'
    )
    
    # Teleop keyboard for manual control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_keyboard',
        prefix='xterm -e',
        output='screen'
    )
    
    return LaunchDescription([
        enable_bridge_arg,
        enable_walking_arg,
        enable_test_arg,
        
        gazebo_launch,
        rosbridge_node,
        walking_controller_node,
        test_controller_node,
        teleop_node
    ])
