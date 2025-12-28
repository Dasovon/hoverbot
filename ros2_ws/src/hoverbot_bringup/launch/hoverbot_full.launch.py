#!/usr/bin/env python3
"""
Complete HoverBot Launch File with Proper Event Handling

Launches all components with proper waiting and error handling
to avoid RPLidar buffer overflow and other timing issues.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    RegisterEventHandler,
    EmitEvent,
    LogInfo
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    bringup_dir = get_package_share_directory('hoverbot_bringup')
    driver_dir = get_package_share_directory('hoverbot_driver')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_enabled = LaunchConfiguration('slam', default='true')
    
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_slam_arg = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Enable SLAM mapping'
    )
    
    # Component 1: HoverBot Driver
    driver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(driver_dir, 'launch', 'hoverbot_driver.launch.py')
        ])
    )
    
    # Component 2: Static Transform Publisher (starts after driver)
    # Wait 3 seconds after driver starts
    tf_publisher = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='static_transform_publisher',
                output='screen',
                arguments=[
                    '--x', '0.1',
                    '--y', '0.0',
                    '--z', '0.1',
                    '--roll', '0.0',
                    '--pitch', '0.0',
                    '--yaw', '0.0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'laser'
                ]
            )
        ]
    )
    
    # Component 3: RPLidar (starts after TF publisher)
    # Wait 5 seconds total from start
    rplidar_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='rplidar_ros',
                executable='rplidar_node',
                name='rplidar_node',
                output='screen',
                parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'frame_id': 'laser',
                    'angle_compensate': True,
                    'scan_mode': 'Sensitivity'
                }],
                # Give it exclusive access to USB
                prefix='nice -n -10'  # Higher priority
            )
        ]
    )
    
    # Component 4: SLAM Toolbox (starts after RPLidar)
    # Wait 8 seconds total from start
    slam_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    os.path.join(bringup_dir, 'config', 'slam_toolbox_params.yaml'),
                    {'use_sim_time': use_sim_time}
                ],
                condition=LaunchConfiguration('slam')
            )
        ]
    )
    
    # Log startup sequence
    startup_log = LogInfo(
        msg='Starting HoverBot components in sequence...'
    )
    
    return LaunchDescription([
        declare_sim_time_arg,
        declare_slam_arg,
        startup_log,
        
        # Launch in order with delays
        driver_node,
        tf_publisher,
        rplidar_node,
        slam_node
    ])
