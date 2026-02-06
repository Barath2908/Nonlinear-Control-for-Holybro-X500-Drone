"""
Full Stack Launch File for Holybro X500 GPS-Denied Autonomy

Launches:
1. VIO pipeline (RealSense T265 or OAK-D)
2. MAVLink Bridge
3. Sliding Mode Controller (optional)
4. RViz2 visualization (optional)

Author: Barath Kumar JK
Date: 2025
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package directories
    pkg_share = FindPackageShare('vio_mavlink_bridge')
    
    # ==================== Launch Arguments ====================
    
    vio_source_arg = DeclareLaunchArgument(
        'vio_source',
        default_value='t265',
        choices=['t265', 'oakd', 'simulation'],
        description='VIO source: t265, oakd, or simulation'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for flight controller'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='Serial baud rate'
    )
    
    enable_controller_arg = DeclareLaunchArgument(
        'enable_controller',
        default_value='false',
        description='Enable sliding mode controller'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    config_dir_arg = DeclareLaunchArgument(
        'config_dir',
        default_value=PathJoinSubstitution([pkg_share, 'config']),
        description='Configuration directory'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )
    
    # ==================== VIO Launch (T265) ====================
    
    t265_launch = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('vio_source'), "' == 't265'"])
        ),
        actions=[
            LogInfo(msg='Launching Intel RealSense T265...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'),
                        'launch',
                        'rs_launch.py'
                    ])
                ]),
                launch_arguments={
                    'camera_name': 'camera',
                    'device_type': 't265',
                    'enable_pose': 'true',
                    'enable_fisheye1': 'false',
                    'enable_fisheye2': 'false',
                    'publish_odom_tf': 'true',
                }.items(),
            ),
        ]
    )
    
    # ==================== VIO Launch (OAK-D) ====================
    
    oakd_launch = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('vio_source'), "' == 'oakd'"])
        ),
        actions=[
            LogInfo(msg='Launching OAK-D VIO...'),
            # Note: Replace with actual OAK-D VIO launch
            # This is a placeholder for depthai-ros VIO node
            Node(
                package='depthai_ros_driver',
                executable='camera_node',
                name='oakd_camera',
                output='screen',
                parameters=[{
                    'i_enable_imu': True,
                    'i_enable_sync': True,
                }],
            ),
        ]
    )
    
    # ==================== Simulation VIO ====================
    
    sim_vio_launch = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('vio_source'), "' == 'simulation'"])
        ),
        actions=[
            LogInfo(msg='Running in simulation mode...'),
            # Fake odometry publisher for testing
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='sim_odom_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            ),
        ]
    )
    
    # ==================== MAVLink Bridge ====================
    
    # Delay bridge startup to ensure VIO is ready
    mavlink_bridge_delayed = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='vio_mavlink_bridge',
                executable='mavlink_bridge_node',
                name='mavlink_bridge',
                namespace='vio',
                output='screen',
                parameters=[
                    PathJoinSubstitution([
                        LaunchConfiguration('config_dir'),
                        'bridge_params.yaml'
                    ]),
                    {
                        'serial_port': LaunchConfiguration('serial_port'),
                        'baudrate': LaunchConfiguration('baudrate'),
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            ),
        ]
    )
    
    # ==================== Sliding Mode Controller ====================
    
    controller_node = Node(
        package='vio_mavlink_bridge',
        executable='sliding_mode_controller',
        name='sliding_mode_controller',
        namespace='control',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_controller')),
        parameters=[
            PathJoinSubstitution([
                LaunchConfiguration('config_dir'),
                'controller_params.yaml'
            ]),
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    
    # ==================== RViz2 ====================
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_rviz')),
        arguments=['-d', PathJoinSubstitution([
            LaunchConfiguration('config_dir'),
            'vio_visualization.rviz'
        ])],
    )
    
    # ==================== Static Transforms ====================
    
    # Camera to base_link transform (adjust based on your setup)
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_base_link_tf',
        arguments=[
            '--x', '0.1',
            '--y', '0.0',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )
    
    return LaunchDescription([
        # Arguments
        vio_source_arg,
        serial_port_arg,
        baudrate_arg,
        enable_controller_arg,
        enable_rviz_arg,
        config_dir_arg,
        log_level_arg,
        
        # VIO sources
        t265_launch,
        oakd_launch,
        sim_vio_launch,
        
        # Static transforms
        camera_tf,
        
        # MAVLink bridge (delayed start)
        mavlink_bridge_delayed,
        
        # Controller
        controller_node,
        
        # Visualization
        rviz_node,
    ])
