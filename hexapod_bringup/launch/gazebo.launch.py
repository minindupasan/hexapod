#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    hexapod_description_dir = get_package_share_directory('hexapod_model_description')
    hexapod_control_dir = get_package_share_directory('hexapod_control')
    
    # Define file paths
    urdf_path = PathJoinSubstitution([
        FindPackageShare('hexapod_model_description'),
        'urdf',
        'hexapod_model.urdf.xacro'
    ])
    
    rviz_config_path = os.path.join(hexapod_description_dir, 'config', 'urdf.rviz')
    controllers_config_path = os.path.join(hexapod_control_dir, 'config', 'hexapod_controllers.yaml')
    bridge_config_path = os.path.join(hexapod_description_dir, 'config', 'ros_gz_bridge_gazebo.yaml')
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file for Gazebo'
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        
        # Start Gazebo Sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py'
                ])
            ]),
            launch_arguments={
                'gz_args': ['-r -v 4 ', LaunchConfiguration('world')],
                'on_exit_shutdown': 'true'
            }.items()
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Spawn robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_hexapod',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'hexapod',
                '-z', '0.5'
            ],
        ),
        
        # ROS-Gazebo Bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{
                'config_file': bridge_config_path,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            output='screen',
            parameters=[
                {'robot_description': Command(['xacro ', urdf_path])},
                controllers_config_path,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ]
        ),
        
        # Joint State Broadcaster
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        ),
        
        # Position Controller
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'hexapod_position_controller'],
            output='screen'
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        )
    ])