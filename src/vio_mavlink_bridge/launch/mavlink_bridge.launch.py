"""
Launch file for VIO-MAVLink Bridge

This launch file starts the MAVLink bridge node that forwards VIO data
to ArduPilot for GPS-denied navigation.

Author: Barath Kumar JK
Date: 2025
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('vio_mavlink_bridge')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for MAVLink communication with flight controller'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='Baud rate for serial communication'
    )
    
    vio_topic_arg = DeclareLaunchArgument(
        'vio_topic',
        default_value='/camera/odom/sample',
        description='Input VIO odometry topic'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'bridge_params.yaml']),
        description='Path to configuration YAML file'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation mode (no serial port)'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # MAVLink Bridge Node
    mavlink_bridge_node = Node(
        package='vio_mavlink_bridge',
        executable='mavlink_bridge_node',
        name='mavlink_bridge',
        namespace='vio',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'vio_topic': LaunchConfiguration('vio_topic'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        remappings=[
            # Add remappings if needed
            # ('/camera/odom/sample', '/t265/odom/sample'),
        ],
    )
    
    # Log startup info
    log_info = LogInfo(
        msg=['Launching VIO-MAVLink Bridge with serial port: ', 
             LaunchConfiguration('serial_port')]
    )
    
    return LaunchDescription([
        # Arguments
        serial_port_arg,
        baudrate_arg,
        vio_topic_arg,
        config_file_arg,
        use_sim_arg,
        log_level_arg,
        
        # Logging
        log_info,
        
        # Nodes
        mavlink_bridge_node,
    ])
